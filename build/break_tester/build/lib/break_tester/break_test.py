import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

import time

breaking_distance = 10.0

lower_speed = 0.5
upper_speed = 10.0
increment = 0.5

class TestBreak(Node):
    def __init__(self):
        super().__init__('break_test')  

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1) 

        # Create Node Variable
        self.gap = 0.0
        self.cur_speed = lower_speed
        self.state = 0 
        self.time_breaked = time.time()
        self.gap = 0.0

        # 0 Resetting
        # 1 Testing
        # 2 Breaking

        # Subscribe to Odom to track speed
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 5) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 5)
        
    def odom_callback(self, msg):
        # RETRIEVE forward/backward velocity from odom message
        velocity_x = msg.twist.twist.linear.x
        
        if (velocity_x <= 0.002 and self.state == 2)
        {
            state = 0

            time_elasped = int(time.time() - self.time_breaked)
            self.get_logger().info(f'Tested Speed {self.cur_speed:6.4f} (m/s) | {time_elasped:6.4f} (s) | {(breaking_distance - self.gap):4.2f} (m)')
            self.cur_speed += increment

            # Initialize message
            msg = AckermannDriveStamped()
            msg.drive.speed = -2.0

            # Publish message to topic (which is initialized in the publisher itself)
            self.drive_pub.publish(msg)
            self.get_logger().info(f"Resetting")

            # Terminate Program
            if(self.cur_speed > upper_speed)
            {
                quit()
            }
        }


    def scan_callback(self, msg):
        # Retrieve Parameters
        self.gap = msg.ranges[540]

        # Break when the distance is reached
        if(self.state == 1 and self.gap < breaking_distance)
        {
            # Initialize message
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0

            # Publish message to topic (which is initialized in the publisher itself)
            self.drive_pub.publish(msg)
            self.get_logger().info(f"Breaking")

            self.time_breaked = time.time()

            self.state = 2
        }
        # Break when the distance is reached
        if(self.state == 0 and self.gap > breaking_distance * 3)
        {
            # Initialize message
            msg = AckermannDriveStamped()
            msg.drive.speed = self.cur_speed

            # Publish message to topic (which is initialized in the publisher itself)
            self.drive_pub.publish(msg)
            self.get_logger().info(f"Initial Point Reached")
            self.get_logger().info(f"Accelerating")

            self.state = 1
        }

def main():
    rclpy.init() 
    node = TestBreak() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        