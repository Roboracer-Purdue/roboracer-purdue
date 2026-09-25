#!/usr/bin/env python3
import numpy as np
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory

class EmergencyBraking(Node):
    def __init__(self):
        super().__init__("emergency_braking")

        self.declare_all_params()
        self.load_params()

        self.scan = None
        self.pose = None
        self.last_pose = None
        self.pose_valid = False
        
        self.is_braking = False

        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9

        self.current_speed = 0.0

        self.last_control_time = self.get_clock().now()
        self.last_state_log_time = 0.0
        self.last_steering = 0.0

        # Subscribe to the scan topic - LIDAR Data
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )

        # Subscribe to the odom topic - Odometry Data
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        # Create publisher for drive topic - Control the Car
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )

        # Create a timer that call control loop repeatedly
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("===================================================")
        self.get_logger().info("SIMPLE DRIVER NODE STARTED")
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
            ("drive_topic", "/drive"),

            ("control_rate_hz", 50.0),

            ("wheelbase", 0.254),
            ("car_width", 0.4),

            ("max_speed", 2.0),
            ("min_speed", 0.0),
            ("steering_limit", 0.42),
        ]

        for name, value in params:
            self.declare_parameter(name, value)

    def load_params(self):
        gp = self.get_parameter

        self.scan_topic = gp("scan_topic").value
        self.odom_topic = gp("odom_topic").value
        self.drive_topic = gp("drive_topic").value

        self.control_rate_hz = float(gp("control_rate_hz").value)

        self.wheelbase = float(gp("wheelbase").value)
        self.car_width = float(gp("car_width").value)

        self.max_speed = float(gp("max_speed").value)
        self.min_speed = float(gp("min_speed").value)
        self.steering_limit = float(gp("steering_limit").value)

    def scan_callback(self, msg):
        # Save scan message from callback to self.scan
        self.scan = msg

    def odom_callback(self, msg):
        # Save odom data from callback to self.current speed and self.pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.current_speed = msg.twist.twist.linear.x
        self.update_pose(x, y, yaw)

    def update_pose(self, x, y, yaw):
        new_pose = np.array([x, y, yaw], dtype=float)

        if self.last_pose is None:
            self.pose_valid = True
            self.pose = new_pose
            self.last_pose = new_pose
            return

        self.pose = new_pose
        self.last_pose = new_pose

    ################
    # CONTROL LOOP #
    ################
    def control_loop(self):
        # Stop car if missing pose/scan data
        if self.scan is None or self.pose is None:
            self.publish_drive(0.0, 0.0)
            return

        # Timing Start
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        # Timing End

        # COMPUTING TTC Collision
        # ------------------------------------------------------
        
        # The Time To Collision (TTC) at scan from angle x is 
        # TTC = r(x) / (v_x cos(x))

        # where r(x) represents LIDAR scan at angle x
        # v_x represent longitudinal velocity (Forward/Backward)

        # 1.  Compute TTC for each scan angle
        TTC = []

        # TODO Update range to cover detection
        for x in range(-90,90):
            rx = self.get_scan_at_angle(x)  # Retrieve LIDAR Scan at angle x
            v_x = self.current_speed        # Retrieve v_x from vehicle's current speed
            
            # TODO Compute TTC Values
            # Make sure you don't accidentally divide by speed of 0

        # 2.  Check if TTC is below a threshold
        # Make sure the value is not negative

        # TODO Check TTC Values
        # Feel free utilize parameter values instead of literal constant

        # 3.  Issue a brake command to drive topic
        if self.is_braking:
            self.publish_drive(0, 0)  # Publish drive command of zero speed and zero steering
            self.log_throttled(1.5, f"EMERGENCY BRAKING.")   # Log Emergency Braking every 1.5 seconds

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)

        self.last_steering = steering
        self.drive_pub.publish(msg)

    ###################
    # -- UTILITIES -- #
    ###################
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
    node = EmergencyBraking()
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