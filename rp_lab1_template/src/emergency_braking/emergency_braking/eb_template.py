#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 

class EmergencyBraking(Node):
    def __init__(self):
        super().__init__("emergency_braking")

        self.declare_all_params()
        self.load_params()

        self.scan = None
        self.current_speed = None

        self.is_braking = False

        self.last_log_time = self.get_clock().now().nanoseconds * 1e-9

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

        self.eb_pub = self.create_publisher(Float32, self.eb_topic, 10)

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
            ("TTC_threshold", 0.3),
            ("control_rate_hz", 50.0),
        ]

        for name, value in params:
            self.declare_parameter(name, value)

    def load_params(self):
        gp = self.get_parameter

        self.scan_topic = gp("scan_topic").value
        self.odom_topic = gp("odom_topic").value
        self.drive_topic = gp("drive_topic").value
        self.eb_topic = gp("eb_topic").value
        self.TTC_threshold = float(gp("TTC_threshold").value)
        self.control_rate_hz = float(gp("control_rate_hz").value)

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

        # 1.  Compute TTC for each scan angle
        TTC = []

        for x in range(-90,91):
            rx = self.get_scan_at_angle(x)  # Retrieve LIDAR Scan at angle x
            v_x = self.current_speed        # Retrieve v_x from vehicle's current speed
            
            if math.isinf(rx) or math.isnan(rx):
                continue

            closing_speed = v_x * math.cos(math.radians(x))

            if closing_speed <= 0:
                continue

            ttc = rx / closing_speed
            if ttc > 0:
                TTC.append(ttc)
            # rx might be 'inf' or 'nan', make sure to handle these
            # Make sure you don't accidentally divide by a projected velocity of 0
            # Keep only valid TTC values that are greater than 0

        # 2.  Check if TTC is below a threshold

        if TTC and min(TTC) < self.TTC_threshold:
            self.is_braking = True
        # Use the TTC_threshold parameter instead of a literal constant

        # 3.  Issue a brake command to drive topic
        if self.is_braking:
            #self.publish_drive(0, 0)  # Publish drive command of zero speed and zero steering

            self.log_throttled(1.5, f"EMERGENCY BRAKING.")   # Log Emergency Braking every 1.5 seconds
                        
            msg = Float32()
            msg.data = 0.0
            self.eb_pub.publish(msg)
            self.is_braking = False

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