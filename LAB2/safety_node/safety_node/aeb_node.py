#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def clamp_nan_inf(x: float, subst: float) -> float:
    if x is None or math.isnan(x) or math.isinf(x):
        return subst
    return x


class AEBNode(Node):
    """
    Automatic Emergency Braking using instantaneous TTC (iTTC).

    Subscribes:
      - /scan (sensor_msgs/LaserScan)
      - /ego_racecar/odom (nav_msgs/Odometry)

    Publishes:
      - /drive (ackermann_msgs/AckermannDriveStamped) with speed=0.0 on hazard

    Parameters:
      - ittc_threshold (float): seconds; brake if any iTTC < threshold
      - min_range (float): meters; ignore ranges below this (noise)
      - fov_deg (float): degrees; forward arc centered at 0 rad to consider
      - use_diff_range_rate (bool): optional alternative dr/dt via diff of consecutive scans
    """

    def __init__(self):
        super().__init__('aeb_node')

        # Parameters (declare with defaults)
        self.declare_parameter('ittc_threshold', 0.5)
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('fov_deg', 180.0)
        self.declare_parameter('use_diff_range_rate', False)

        self.ittc_threshold = float(self.get_parameter('ittc_threshold').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.use_diff_range_rate = bool(self.get_parameter('use_diff_range_rate').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, qos)

        # Publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Vehicle state
        self.vx = 0.0  # longitudinal speed (m/s)

        # For diff method
        self.prev_scan: Optional[LaserScan] = None

        self.get_logger().info(
            f'AEBNode started — ittc_threshold={self.ittc_threshold}s, '
            f'min_range={self.min_range}m, fov={self.fov_deg}deg, '
            f'use_diff_range_rate={self.use_diff_range_rate}'
        )

    def odom_cb(self, msg: Odometry):
        # Longitudinal velocity in car frame (approx: use x in Odom frame used by sim)
        vx = msg.twist.twist.linear.x
        self.vx = clamp_nan_inf(vx, 0.0)

    def scan_cb(self, scan):
        import math
        n = len(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        half_fov_rad = math.radians(self.fov_deg) / 2.0
        forward_min = -half_fov_rad
        forward_max = +half_fov_rad

        hazard = False
        ittc_min = float('inf')

        for i in range(n):
            r = scan.ranges[i]
            if math.isinf(r) or r < self.min_range:
                continue
            theta = angle_min + i * angle_inc
            if not (forward_min <= theta <= forward_max):
                continue

            # correct projection sign
            drdt = -self.vx * math.cos(theta)

            neg_drdt = max(-drdt, 0.0)
            if neg_drdt <= 1e-6:
                ittc = float('inf')
            else:
                ittc = r / neg_drdt

            ittc_min = min(ittc_min, ittc)
            if ittc < self.ittc_threshold:
                hazard = True

        # ----- STATE TRACKER -----
        # interpret current situation
        if ittc_min == float('inf'):
            state = "SAFE ✅  (no approach)"
        elif ittc_min < self.ittc_threshold:
            state = f"BRAKING 🛑  (iTTC={ittc_min:.2f}s < {self.ittc_threshold:.2f}s)"
        elif ittc_min < self.ittc_threshold * 2.5:
            state = f"CAUTION ⚠️  (iTTC={ittc_min:.2f}s)"
        else:
            state = f"SAFE ✅  (iTTC={ittc_min:.2f}s)"

        self.get_logger().info(state)

        # ----- BRAKE COMMAND -----
        if hazard:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.drive_pub.publish(msg)
            self.get_logger().warn(
                f"AEB BRAKE! min_iTTC={ittc_min:.3f}s < {self.ittc_threshold:.3f}s"
            )



def main(args=None):
    rclpy.init(args=args)
    node = AEBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
