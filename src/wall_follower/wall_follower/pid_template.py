#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 

class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.declare_all_params()
        self.load_params()

        self.scan = None
        self.current_speed = None
        self.target_speed = self.max_speed # Set target speed to max

        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_eb_time = self.get_clock().now().nanoseconds * 1e-9

        # TODO define your new node variables here

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

        # Subscribe to the Emergency Braking Node - Float32
        self.eb_sub = self.create_subscription(
            Float32,
            self.eb_topic,
            self.eb_callback,
            10,
        )
        # Create publisher for drive topic - Control the Car
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )

        # TODO Create a publisher for /eb topic

        # Create a timer that call control loop repeatedly
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("===================================================")
        self.get_logger().info("EMERGENCY BRAKING NODE STARTED")
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
            ("eb_topic", "/eb"),

            ("control_rate_hz", 50.0),

            # New parameters
            ("eb_topic", "/eb"),    # Emergency brake topic
            ("eb_stall_time", 3.0), # Time required to reset target speed (seconds)

            ("max_speed", 2.0),   

            # PID
            # TODO Define K Constants for PID Controllers
        ]

        for name, value in params:
            self.declare_parameter(name, value)

    def load_params(self):
        gp = self.get_parameter

        self.scan_topic = gp("scan_topic").value
        self.odom_topic = gp("odom_topic").value
        self.drive_topic = gp("drive_topic").value

        self.control_rate_hz = float(gp("control_rate_hz").value)

        # New parameters
        self.eb_topic = gp("eb_topic").value
        self.eb_stall_time = gp("eb_stall_time").value

        self.max_speed = gp("max_speed").value

        # PID
        # TODO Assign K Constants for PID Controllers

    def eb_callback(self, msg):
        # Set target speed down to 0 to emergency brake the car
        self.target = msg.data
        self.last_eb_time = self.get_clock().now().nanoseconds * 1e-9

    def scan_callback(self, msg):
        # Save scan message from callback to self.scan
        self.scan = msg

    def odom_callback(self, msg):
        # Save longitudinal velocity from odometry
        self.current_speed = msg.twist.twist.linear.x

    ################
    # CONTROL LOOP #
    ################
    def control_loop(self):
        # Stop car if LiDAR or odometry data has not been received
        if self.scan is None or self.current_speed is None:
            self.publish_drive(0.0, 0.0)
            return


        # COMPUTING TTC Collision
        # ------------------------------------------------------
        
        # The Time To Collision (TTC) at scan from angle x is 
        # TTC = r(x) / (v_x cos(x))

        # where r(x) represents LIDAR scan at angle x
        # v_x represents longitudinal velocity (Forward/Backward)

        # Note that TTC only makes sense when v_x cos(x) > 0.
        # Otherwise, the obstacle is not being approached along that LiDAR ray.
        # How would you handle this?

        # 1.  Compute Errors
        
        # TODO Compute Proportional Error

        # TODO Compute Derivative Error

        # TODO Compute Integral Error

        # 2.  Compute Steering
        steering = 0


        # 3.  Compute Proper Speed

        # RESTART LOGIC - Reset the car target speed after emergency brake worn off
        if self.get_clock().now().nanoseconds * 1e-9 - self.last_eb_time > self.eb_stall_time:
            self.target_speed = self.max_speed

        # The following is a simple speed control based on current steering
        speed = 0

        if abs(steering) > 0.4:
            # Low Gear : Large Steering
            speed = 0.5 
        elif abs(steering) > 0.15:
            # Mid Gear : Slight Steering
            speed = 0.7 * self.target_speed
        else:
            # High Gear : Small Steering
            speed = self.target_speed

        # 4.  Drive at target speed and computed steering
        
        # TODO Publish speed and steering drive command
        # TODO Use self.log_throttled() to log current speed and steering

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