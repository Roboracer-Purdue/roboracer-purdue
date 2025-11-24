import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import time

# BREAKING TIME
# Quadratic Formula av + b
a = 105.38
b = 333.21

# Output Limiter
TTC_inf_value = 1000
TTC_zero_value = 0.00001

class EmergencyBreak(Node):
    def __init__(self):
        super().__init__('emergency_break')  
        
        # Declare Paramters
        self.declare_parameter('barrier_width', 2.0)

        # Initialize Node Variables
        self.velocity_x = 0
        self.beam_velocity = [0.0] * 1080
        self.TTC = [0.0] * 1080

        self.brake_state = 0
        self.brake_start = time.time()

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1) 

        # Subscribe to odom to update car velocity
        self.odom_sub = self.create_subscription(Odometry, "/roboworks/odom", self.odom_callback, 5) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 5)

        # Create a timer to call a function that check for potential collsion

    def odom_callback(self, msg):
        # RETRIEVE forward/backward velocity from odom message
        self.velocity_x = msg.twist.twist.linear.x
        # self.get_logger().info(f'Updated X-Velocity to {velocity_x:.3f} (m/s)')
        
    def scan_callback(self, msg):
        # Retrieve Parameters
        barrier_width = self.get_parameter('barrier_width').get_parameter_value().double_value

        '''
        # Print all the information of scan
        self.get_logger().info(f'Received scan with {len(msg.ranges)} ranges.')

        self.get_logger().info(f'Start angle of the scan: {msg.angle_min} (rad)')
        self.get_logger().info(f'End angle of the scan: {msg.angle_max} (rad)')
        self.get_logger().info(f'Angular distance between measurements: {msg.angle_increment} (rad)')
        
        self.get_logger().info(f'Time between measurements: {msg.time_increment} (s)')
        self.get_logger().info(f'Time between scans: {msg.scan_time} (s)')
        
        self.get_logger().info(f'Minimum range value: {msg.range_min} (m)')
        self.get_logger().info(f'Maximum range value: {msg.range_max} (m)')
        '''
        # Example: print the first range value
        '''
        if len(msg.ranges) > 0:
            self.get_logger().info(f'First range: {msg.ranges[0]}')
            self.get_logger().info(f'Last range: {msg.ranges[-1]}')
        '''
 
        # Calculate the velocity along each of the scan beam
        cur_angle = msg.angle_min
        for i in range(len(msg.ranges)):
            self.beam_velocity[i] = np.cos(cur_angle) * self.velocity_x
            cur_angle += msg.angle_increment
        
        # Calculate the time to colision on each of the beam
        cur_angle = msg.angle_min
        for i in range(len(msg.ranges)):
            # Check if the TTC is in the break barrier
            if abs(msg.ranges[i] * np.sin(cur_angle)) < barrier_width / 2:
                if self.beam_velocity[i] != 0:
                    self.TTC[i] = min((msg.ranges[i]) / max(self.beam_velocity[i], TTC_zero_value), TTC_inf_value)
                else:
                    self.TTC[i] = TTC_inf_value
            else:
                self.TTC[i] = TTC_inf_value
            cur_angle += msg.angle_increment
        # Log Beam reading, Velocity, and TTC to terminals
        
        for i in range(len(msg.ranges)):
            self.get_logger().info(f'Beam No. {i:4d}: {msg.ranges[i]:8.2f} {self.beam_velocity[i]:8.2f} {self.TTC[i]:8.2f}')
        

        # Publish to drive if a TTC is too low
        # min(self.TTC) < self.velocity_x / breaking_deceleration

        # CALCULATE TIME TO BRAKE
        TTB = a*self.velocity_x + b
        TTB /= 1000 # convert to second

        #self.get_logger().info(f"STATUS: TTC {min(self.TTC):.5f} TTB {TTB:.5f} ")

        if min(self.TTC) < TTB:
            # Initialize message
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.brake_start = time.time()
            self.brake_state = 1

            # Publish message to topic (which is initialized in the publisher itself)
            self.drive_pub.publish(msg)
            self.get_logger().info(f"EMERGENCY BREAKING: TTC {min(self.TTC):.5f} TTB {TTB:.5f} ")
        
        if self.brake_state == 1 and self.velocity_x < 0.001:
            self.brake_state = 0
            self.get_logger().info(f"Actual Brake Time: TTB {time.time()-self.brake_start:.5f} ")

def main():
    rclpy.init() 
    node = EmergencyBreak() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        