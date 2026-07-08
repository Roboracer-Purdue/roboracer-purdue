#!/usr/bin/env python3

import csv
import math
from enum import Enum
from dataclasses import dataclass

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
import os

class DualControllerNode(Node):
    def __init__(self):
        super().__init__("dual_controller")

        self.declare_all_params()
        self.load_params()

        self.is_reversing = False

        self.scan = None
        self.pose = None
        self.last_pose = None
        self.pose_valid = False

        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9

        self.current_speed = 0.0

        self.last_control_time = self.get_clock().now()
        self.last_state_log_time = 0.0

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        if self.use_amcl_pose:
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.pose_callback,
                10,
            )
            self.get_logger().info(f"Using localized pose topic: {self.pose_topic}")

        else:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                10,
            )
            self.get_logger().info(f"Using odom topic: {self.odom_topic}")

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("===================================================")
        self.get_logger().info("DUAL CONTROLLER NODE STARTED")
        self.get_logger().info(f"Publishing drive commands to: {self.drive_topic}")
        self.get_logger().info("===================================================")


    '''
    TO ADD NEW PARAMETERS
    1. Put new tuple in params list below
    2. Load the parameter to node in "load_params()" function
    '''
    def declare_all_params(self):
        params = [
            ("scan_topic", "/scan"),
            ("odom_topic", "/ego_racecar/odom"),
            ("pose_topic", "/amcl_pose"),
            ("use_amcl_pose", False),
            ("drive_topic", "/drive"),

            ("control_rate_hz", 30.0),

            ("wheelbase", 0.254),

            ("max_speed", 2.0),
            ("min_speed", 0.35),
            ("accel_limit", 3.5),
            ("decel_limit", 2.5),

            ("steering_limit", 0.42),
            ("steering_rate_limit", 1.8),

            # CONSTANT RADIUS ARC - GAP FOLLOW
            # - Choose the middle of the gap closest to the center
            # - If no gap found, keep forward
            # WALL RADIUS ARC - GAP FOLLOW
            # - same as the last one, but the arc_radius is added onto distance to wall
            ("gf_min_angle", -135.0), # in Degrees
            ("gf_max_angle", 135.0), # in Degrees
            ("gf_angle_sep", 0.5), # angle separation between scans in degrees
            ("gf_min_gap_width", 3.0), # in Degrees
            ("gf_arc_radius", 1.55), # In meters # (1.5 Is sweet, but unable to clear)

        ]

        for name, value in params:
            self.declare_parameter(name, value)

    def load_params(self):
        gp = self.get_parameter

        self.scan_topic = gp("scan_topic").value
        self.odom_topic = gp("odom_topic").value
        self.pose_topic = gp("pose_topic").value
        self.use_amcl_pose = bool(gp("use_amcl_pose").value)
        self.drive_topic = gp("drive_topic").value

        self.control_rate_hz = float(gp("control_rate_hz").value)

        self.wheelbase = float(gp("wheelbase").value)

        self.max_speed = float(gp("max_speed").value)
        self.min_speed = float(gp("min_speed").value)
        self.accel_limit = float(gp("accel_limit").value)
        self.decel_limit = float(gp("decel_limit").value)

        self.steering_limit = float(gp("steering_limit").value)
        self.steering_rate_limit = float(gp("steering_rate_limit").value)

        self.gf_min_angle = float(gp("gf_min_angle").value)
        self.gf_max_angle = float(gp("gf_max_angle").value)
        self.gf_min_gap_width = float(gp("gf_min_gap_width").value)
        self.gf_arc_radius = float(gp("gf_arc_radius").value)
        self.gf_angle_sep = float(gp("gf_angle_sep").value)

    def scan_callback(self, msg):
        #self.log_throttled(2.0, "Received Scan")
        self.scan = msg

    def odom_callback(self, msg):
        #self.log_throttled(2.0, "Received Odom")

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_speed = msg.twist.twist.linear.x
        self.update_pose(x, y, yaw)

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.update_pose(x, y, yaw)

    def update_pose(self, x, y, yaw):
        new_pose = np.array([x, y, yaw], dtype=float)

        if self.last_pose is None:
            self.pose_valid = True
            self.pose = new_pose
            self.last_pose = new_pose
            return
            
        '''
        dx = x - self.last_pose[0]
        dy = y - self.last_pose[1]
        dyaw = self.angle_diff(yaw, self.last_pose[2])

        
        if math.hypot(dx, dy) > self.pose_jump_threshold:
            self.get_logger().warn("Pose jump detected!!!")
            self.pose_valid = False
        elif abs(dyaw) > self.yaw_jump_threshold:
            self.get_logger().warn("Yaw jump detected!!!")
            self.pose_valid = False
        else:
            self.pose_valid = True
        '''

        self.pose = new_pose
        self.last_pose = new_pose

    ################
    # CONTROL LOOP #
    ################
    def control_loop(self):
        if self.scan is None or self.pose is None:
            self.publish_drive(0.0, 0.0)
            return

        # Timing Start
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        # Timing End

        # Compute Steering
        steering = self.constant_radius_gap_follow(
            self.gf_min_angle,
            self.gf_max_angle,
            self.gf_angle_sep,
            self.gf_arc_radius,
            self.gf_min_gap_width
        )
        # If too close to left wall, steer outward
        #if self.get_scan_at_angle(90.0) < 0.2:
        #    steering -= 0.1

        # steering += (self.get_scan_at_angle(90.0)-0.7) / 5.0

        # Compute Speed
        dist_to_front = self.get_forward_distance()
        safety_dist = self.current_speed / 4.0

        speed = (max(dist_to_front - safety_dist, 0) ** 0.5) * 4.0 # 3.0
        
        # If reversing, flip steering
        # if speed < 0:
        #    steering = -steering

        ### If going fast, limit steering
        #if self.current_speed > 3.0:
        #    steering = self.clamp(steering, -0.01, 0.01)

        self.log_throttled(0.5, f"Driving at {speed:.2f} toward {steering:.2f}")

        # Publish drive command
        self.publish_drive(speed, steering)

        # Log Current Speed and Steering
        self.print_status()

    def wall_radius_gap_follow(self, min_angle, max_angle, angle_sep, radius, min_gapwidth):
        wall_dist = max(self.get_scan_at_angle(-90.0),self.get_scan_at_angle(90.0))
        return self.constant_radius_gap_follow(min_angle, max_angle, angle_sep, radius + wall_dist, min_gapwidth)

    def constant_radius_gap_follow(self, min_angle, max_angle, angle_sep, radius, min_gapwidth):
        # Return steering angle
        
        # Lower radius if forward distance is too low
        forward = self.get_forward_distance()
        radius = min(radius, forward+0.08)

        # HARD TURN, turn immediately if there is big gap on left first ### or right
        
        #if self.get_scan_at_angle(60.0) > forward *1.2:
        if self.get_scan_at_angle(45.0) > forward *1.1:
            return self.clamp(1.0, -self.steering_limit, self.steering_limit)
        elif self.get_scan_at_angle(-45.0) > forward *1.1:
            return self.clamp(-1.0, -self.steering_limit, self.steering_limit)

        

        # Check all scan in range to map gaps
        gaps = []
        
        first_gap_angle = -1
        
        deg = min_angle
        while deg <= max_angle:
            #self.get_logger().info(f"{deg} {max_angle}")
            if self.get_scan_at_angle(deg) > radius:
                first_gap_angle = deg
            
                break
            deg += angle_sep

        # If no gap, keep cruising straight
        if first_gap_angle == -1:
            return 0.0

        last_gap_start = first_gap_angle
        deg = first_gap_angle
        deg_weight = 1.0
        while deg <= max_angle:
            if self.get_scan_at_angle(deg) <= radius:
                if last_gap_start != None and deg - last_gap_start > min_gapwidth:
                    # Append middle of gap
                    gaps.append((deg * deg_weight + last_gap_start) / (1.0 + deg_weight))
                last_gap_start = None
            else:
                if last_gap_start == None:
                    last_gap_start = deg
            deg += angle_sep

        # Clearing
        if last_gap_start != None and deg - last_gap_start > min_gapwidth:
            # Append middle of gap
            gaps.append((deg + last_gap_start) / 2.0)
            last_gap_start = None
        
        if len(gaps) == 0:
            return 0.0

        # 45 Is the targeting angle
        target_angle = min(gaps, key=lambda x:abs(x))
        
        #self.get_logger().info(f"Target Angle: {target_angle}")

        # Use pure pursuit formula to drive to target angle
        dy = math.sin(math.radians(target_angle))

        L = radius
        curvature = 2 * dy / (L * L)
        steering = math.atan(self.wheelbase * curvature) * 4.0 # 4.0

        # Track specific adjustment: WHen turning hard, turn harder
        #if abs(steering) > 0.3:
        #    steering *= 5.0


        #steering = target_angle
        return self.clamp(steering, -self.steering_limit, self.steering_limit)


    def print_status(self):
        self.log_throttled(
            1.0,
            (
                f"Pose=({self.pose[0]:.2f}, {self.pose[1]:.2f}) "
                f"Speed={self.current_speed:.2f} "
            )
        )

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.drive_pub.publish(msg)


    ###################
    # -- UTILITIES -- #
    ###################
    def get_forward_distance(self):
        return min(self.get_scan_at_angle(-1.0),self.get_scan_at_angle(-0.0),self.get_scan_at_angle(1.0))

    def get_scan_at_angle(self, angle):
        """
        Returns the LiDAR range closest to the requested angle.

        Parameters
        ----------
        angle : float
            Angle in degrees relative to the LiDAR frame.
            0      = straight ahead
            +90  = left
            -90  = right
        """

        if self.scan is None:
            return float("nan")

        angle = math.radians(angle)

        index = int(round(
            (angle - self.scan.angle_min) /
            self.scan.angle_increment
        ))

        index = max(0, min(index, len(self.scan.ranges) - 1))

        return self.scan.ranges[index]
        
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def log_throttled(self, period, message):
        now = self.get_clock().now().nanoseconds * 1e-9

        if now - self.last_log_time >= period:
            self.get_logger().info(message)
            self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = DualControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()