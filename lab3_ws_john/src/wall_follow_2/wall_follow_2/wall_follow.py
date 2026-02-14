#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import bisect

from typing import List, Tuple, Optional

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")

        # Declare Parameters
        self.declare_parameter("scan_topic", "/scan")
        # car: /ackermann_cmd
        # sim: /drive
        self.declare_parameter("drive_topic", "/drive") 
        # car: /odom
        # sim: /ego_racecar/odom
        self.declare_parameter("odom_topic", "/ego_racecar/odom")

        # For clamping laserscan ranges
        self.declare_parameter("front_fov_deg", 200.0)
        self.declare_parameter("range_clip_max", 10.0)
        self.declare_parameter("range_clip_min", 0.05)

        # Steerings
        self.declare_parameter("steer_limit_rad", 0.40)
        self.declare_parameter("steer_smooth_alpha", 0.35)

        # Speed control
        self.declare_parameter("speed_min", 0.5) # For turning
        self.declare_parameter("speed_max", 2.0) # For gassin'

        # Wallfollowing Parameters
        self.declare_parameter("target_distance", 0.3)
        self.declare_parameter("look_ahead", 1.0)
        self.declare_parameter("beam_a_angle", -60.0)
        self.declare_parameter("beam_b_angle", -90.0)
        self.declare_parameter("K_p", 2.0)
        self.declare_parameter("K_i", 0.0)
        self.declare_parameter("K_d", 0.0)

        # Automatic Brake
        self.declare_parameter("ttc_emergency", 0.5)
        self.declare_parameter("ttc_slow", 0.7)

        # --- State ---
        self.prev_steer = 0.0
        self.velocity_x = 0.0

        scan_topic = self.get_parameter("scan_topic").value
        drive_topic = self.get_parameter("drive_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, scan_topic, self.odom_callback, 7)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 5)

        self.get_logger().info(f"Wall_Follow: sub={scan_topic}, {odom_topic} pub={drive_topic}")
    
    def odom_callback(self, msg: Odometry):
        if not msg:
            return
        
        self. velocity_x = msg.twist.twist.linear.x

    def scan_callback(self, msg: LaserScan):
        ranges, angles = self.get_front_sector(msg)
        if not ranges:
            return

        cleaned = self.clean_ranges(ranges, msg.range_min, msg.range_max)

        # Forward distance (small cone)
        forward_dist = self.forward_distance(cleaned, angles, half_angle_deg=7.0)

        # Steering
        steer = self.follow_wall(cleaned, angles)

        # Speed planning
        speed = self.plan_speed(cleaned, angles, steer)

        # Use planned speed if no callback
        odom_topic = self.get_parameter("odom_topic").value
        if odom_topic == "":
            self.velocity_x = speed

        # TTC safety override
        ttc = self.estimate_ttc(cleaned, angles, self.velocity_x)
        vmin = float(self.get_parameter("speed_min").value)
        ttc_emergency = float(self.get_parameter("ttc_emergency").value)
        ttc_slow = float(self.get_parameter("ttc_slow").value)

        if ttc is not None:
            if ttc <= ttc_emergency:
                speed = 0.0
            elif ttc <= ttc_slow:
                speed = vmin

        # Smooth steering
        alpha = float(self.get_parameter("steer_smooth_alpha").value)
        steer_limit = float(self.get_parameter("steer_limit_rad").value)
        steer = clamp(steer, -steer_limit, steer_limit)
        steer = alpha * steer + (1.0 - alpha) * self.prev_steer
        self.prev_steer = steer

        #self.get_logger().info(f'DRIVE steer: {steer} speed: {speed}')
        self.publish_drive(speed, steer)

    # ---------- LiDAR helpers ----------
    # Process a list of angle and readings
    def get_front_sector(self, msg: LaserScan) -> Tuple[List[float], List[float]]:
        fov_deg = float(self.get_parameter("front_fov_deg").value)
        fov = math.radians(fov_deg)

        out_ranges: List[float] = []
        out_angles: List[float] = []

        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            # map to [-pi, pi] so "front" is near 0
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) <= (fov * 0.5):
                out_ranges.append(r)
                out_angles.append(ang)

        return out_ranges, out_angles

    # Cleaning scan data (value above max are replaced with infinite)
    def clean_ranges(self, ranges: List[float], rmin: float, rmax: float) -> List[float]:
        clip_max = float(self.get_parameter("range_clip_max").value)
        clip_min = float(self.get_parameter("range_clip_min").value)

        cleaned: List[float] = []
        for r in ranges:
            if r is None or not math.isfinite(r):
                min(rmax, clip_max)
                continue
            if r < max(rmin, clip_min):
                cleaned.append(max(rmin, clip_min))
            elif r >= min(rmax, clip_max):
                cleaned.append(float("inf"))
            else:
                cleaned.append(r)
        return cleaned

    def publish_drive(self, speed: float, steer: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    # Find best steering
    def follow_wall(self, ranges: List[float], angles: List[float]) -> float:
        target_distance = float(self.get_parameter("target_distance").value)
        beam_a_angle = float(self.get_parameter("beam_a_angle").value)
        beam_b_angle = float(self.get_parameter("beam_b_angle").value)
        look_ahead = float(self.get_parameter("look_ahead").value)
        K_p = self.get_parameter('K_p').get_parameter_value().double_value
        K_i = self.get_parameter('K_i').get_parameter_value().double_value
        K_d = self.get_parameter('K_d').get_parameter_value().double_value

        # Retrive a and b laser scan
        a_id = bisect.bisect_left(angles, math.radians(beam_a_angle))
        b_id = bisect.bisect_left(angles, math.radians(beam_b_angle))
        a = ranges[a_id]
        b = ranges[b_id]

        theta = beam_b_angle - beam_a_angle

        #self.get_logger().info(f'len angle {len(angles)} ranges {len(ranges)}')
        self.get_logger().info(f'a_id: {a_id} b_id: {b_id} a: {a:.3f}')
        #self.get_logger().info(f'a_angle: {beam_a_angle} b_angle: {beam_b_angle}')
        #self.get_logger().info(f'a: {a} b: {b}')
        #self.get_logger().info(f'min: {min(angles)} max: {max(angles)}')


        # Calculate alpha
        alpha = np.arctan((a * np.cos(theta) - b)/(a * np.sin(theta)))
        #self.get_logger().info(f'Alpha: {alpha:.3f}')

        # Caculate Dt and Dt1
        Dt = b * np.cos(alpha)
        Dt1 = Dt + look_ahead * np.sin(alpha)
        #self.get_logger().info(f'Dt: {Dt:.3f} Dt1: {Dt1:.3f} ')

        # Compute PID steering angles
        et = target_distance - Dt
        et1 = target_distance - Dt1

        t = look_ahead / max(self.velocity_x, 0.00001)

        term1 = et1
        term2 = (et1 + et) / 2 * t
        term3 = (et - et1)

        ut = (term1 * K_p + term2 * K_i + term3 * K_d)

        return ut

    def plan_speed(self, ranges: List[float], angles: List[float], steer: float) -> float:
        vmin = float(self.get_parameter("speed_min").value)
        vmax = float(self.get_parameter("speed_max").value)

        # Decide Speed based on distance to obstacle and Steer
        '''
        fwd = self.forward_distance(ranges, angles, half_angle_deg=10.0)

        if fwd is None:
            fwd = 0.8

        v_dist = vmin + dist_gain * math.log(1.0 + max(0.0, fwd))
        v_turn = vmax / (1.0 + turn_scale * abs(steer))

        v = min(v_dist, v_turn)
        return clamp(v, vmin, vmax)
        '''
        # CONDITIONAL SPEED DECISION
        if abs(steer) > 0.5:
            v = vmin
        else:
            v = vmax

        return v

    def forward_distance(self, ranges: List[float], angles: List[float], half_angle_deg: float) -> Optional[float]:
        half = math.radians(half_angle_deg)
        vals = [r for r, a in zip(ranges, angles) if abs(a) <= half and math.isfinite(r)]
        if not vals:
            return None
        return min(vals)

    def estimate_ttc(self, ranges: List[float], angles: List[float], speed: float) -> Optional[float]:
        if speed <= 0.05:
            return None

        half = math.radians(20.0)
        best = None
        for r, a in zip(ranges, angles):
            if not math.isfinite(r):
                continue
            if abs(a) > half:
                continue
            closing = speed
            if closing < 0.05:
                continue
            ttc = r * max(0.0, math.cos(a)) / speed
            if best is None or ttc < best:
                best = ttc
        return best
    
def main():
    rclpy.init()
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
