#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32, Float64

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.
        # TODO: create ROS subscribers and publishers.
        self.publisherdrive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.subscriberlaser = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.subscriberodom = self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.publishereb = self.create_publisher(Float32, '/eb', 10)
        
    def odom_callback(self, odom_msg):  #break function when obstacle is seen at a distance within breaking zone
        # TODO: update current speed
        self.speed = 0.0
        velocity_msg = odom_msg.twist.twist.linear
        self.speed = velocity_msg.x
    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        in_breaking_zone = False
        #calculating range rate
        i = 0
        #ttc = []                    #array of ttc's 
        smallest_ttc = 10000        #keeps track of smallest ttc
        r_values = scan_msg.ranges  #array of ranges
        for i, current_range in enumerate(r_values):
            beam_angle= scan_msg.angle_min + (scan_msg.angle_increment * i)
            range_rate= self.speed * np.cos(beam_angle)
            #Check for valid range values and beam_angles
            if not np.isfinite(current_range):
                continue
            if (current_range < scan_msg.range_min) or (current_range > scan_msg.range_max):
                continue
            if not np.isfinite(beam_angle):
                continue 
            if (self.speed < 0): continue
            

            ttc_value = current_range / max(range_rate, 0.01)
            if ttc_value < smallest_ttc: smallest_ttc = ttc_value
            #ttc.append(ttc_value)
        # TODO: publish command to brake
        self.get_logger().info(str(smallest_ttc))
        
        if smallest_ttc < 0.6:
            msg = Float32()
            msg.data = 0.0
            self.publishereb.publish(msg)

            # drive_msg = AckermannDriveStamped()
            # drive_msg.drive.speed = 0.0
            # self.publisherdrive.publish(drive_msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()