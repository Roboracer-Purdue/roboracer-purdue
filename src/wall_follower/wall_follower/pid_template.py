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
        self.last_ctrl_time = self.get_clock().now().nanoseconds * 1e-9

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

        # Create a timer that call control loop repeatedly
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.control_loop,
        )

        self.get_logger().info("===================================================")
        self.get_logger().info("WALL FOLLOWER NODE STARTED")
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

            # New parameters
            ("eb_topic", "/eb"),    # Emergency brake topic
            ("max_speed", 2.0),   

            # PID
            # TODO Define Angle A,B (Degrees) for PID Controller
            # TODO Define Lookahead Distance (Meters) for PID Controller
            # TODO Define Target Spacing (meters) for PID Controller

            # TODO Define K Constants for PID Controller
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
        self.max_speed = gp("max_speed").value

        # PID
        # TODO Assign Angle A,B (Degrees) for PID Controller
        # TODO Assign Lookahead Distance (Meters) for PID Controller
        # TODO Assign Target Spacing (meters) for PID Controller

        # TODO Assign K Constants for PID Controller

    def eb_callback(self, msg):
        # Set target speed down to 0 to emergency brake the car
        self.target_speed = msg.data

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

        # dt might come in handy
        cur_time = self.get_clock().now().nanoseconds * 1e-9
        dt = cur_time - self.last_ctrl_time
        self.last_ctrl_time = cur_time

        # COMPUTING Distance From Wall
        # ------------------------------------------------------
        # How do we measure how far is our car from the wall?
        # One easy answer is measuring the LIDAR distance directly to the left (90 degs);
        # however, the car is not always parallel to the wall.
        # 
        # The car heading will make an angle 'alpha' from the line parallel to the wall.
        # Check the lab manual on how to compute alpha, and how to use it to compute distance

        # 1.  Compute Heading and Distance

        # TODO read LIDAR scan at angle A
        a = 0
        # TODO read LIDAR scan at angle B
        b = 0

        # TODO compute alpha
        alpha = 0

        # TODO compute distance D_t
        D_t = 0

        # TODO compute predicted distance D_n
        # Hint: Use newly defined parameter self.wf_lookahead
        D_n = 0

        # COMPUTING Steering
        # ------------------------------------------------------
        # PID stands for Proportional - Integral - Derivative
        # A wall follow want to cruise on a line drawn a certain distance from the wall
        # Proportional - The deviation from that line is e(t)
        #   Positive error: Too far from left wall -> steer left
        #   Negative error: Too close to left wall -> steer right
        # Derivative - How fast e(t) is changing 
        #   If we are approaching the line too fast, we want to steer away from it a bit
        # Integral - Cumulative e(t)
        #   Helps optimize steering by trying to counteract accumulated error.
        # 
        # By combining these errors, we can build a PID controller that manage steering
        # steer = K_p * P_Error + K_d * D_Error + K_i * I_Error
        #
        # There is one issue, if we use the current distance from the wall to calculate steering,
        # our response might be too late. Car take some amount of time to steer/slow down.
        #
        # Hence, for this PID lab, use the predicted distance D_n to calculate errors.
        
        # 2.  Compute Errors
        
        # TODO Compute Proportional Error
        P_Error = 0

        # TODO Compute Derivative Error
        # Hint: it might be helpful to initialize more node variable
        #       self.last_error
        D_Error = 0

        # TODO Compute Integral Error
        # Hint: it might be helpful to initialize more node variable
        #       self.acc_error
        # Note: The first few seconds of running are spent on getting close to the race line
        #       Accumulator should be disabled during the first three seconds
        I_Error = 0

        # 3.  Compute Steering
        steering = 0

        # 4.  Compute Proper Speed

        # The following is a simple speed control based on current steering
        speed = 0

        if abs(steering) > 0.4:
            # Low Gear : Large Steering
            speed = 0.3 * self.target_speed
        elif abs(steering) > 0.15:
            # Mid Gear : Slight Steering
            speed = 0.7 * self.target_speed
        else:
            # High Gear : Small Steering
            speed = self.target_speed

        # 5.  Drive at target speed and computed steering
        
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
    node = WallFollower()
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