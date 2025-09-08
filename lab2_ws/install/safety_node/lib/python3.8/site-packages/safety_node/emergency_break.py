import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

num_beam = 1080

# BREAKING
breaking_deceleration = 0.5 # m/s^2
barrier_width = 0.3; # Car width 23.19 cm
TTB = 0.6

# Output Limiter
TTC_inf_value = 1000
TTC_zero_value = 0.00001
start_beam_index = 0
end_beam_index = 1079

class EmergencyBreak(Node):
    def __init__(self):
        super().__init__('emergency_break')  
        
        # Declare Paramters
        self.declare_parameter('breaking_deceleration', 0.5)
        self.declare_parameter('TTB', 0.4)
        self.declare_parameter('barrier_width', 0.3)

        # Initialize Node Variables
        self.velocity_x = 0
        self.beam_velocity = [0.0] * num_beam
        self.TTC = [0.0] * num_beam

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1) 

        # Subscribe to odom to update car velocity
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 5) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 5)

        # Create a timer to call a function that check for potential collsion

    def odom_callback(self, msg):
        # RETRIEVE forward/backward velocity from odom message
        self.velocity_x = msg.twist.twist.linear.x
        # self.get_logger().info(f'Updated X-Velocity to {velocity_x:.3f} (m/s)')
        
    def scan_callback(self, msg):
        # Retrieve Parameters
        breaking_deceleration = self.get_parameter('breaking_deceleration').get_parameter_value().double_value
        TTB = self.get_parameter('TTB').get_parameter_value().double_value
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
                    self.TTC[i] = min(msg.ranges[i] / max(self.beam_velocity[i], TTC_zero_value), TTC_inf_value)
                else:
                    self.TTC[i] = TTC_inf_value
            else:
                self.TTC[i] = TTC_inf_value
            cur_angle += msg.angle_increment
        # Log Beam reading, Velocity, and TTC to terminals
        '''
        for i in range(start_beam_index, end_beam_index + 1):
            self.get_logger().info(f'Beam No. {i:4d}: {msg.ranges[i]:8.2f} {self.beam_velocity[i]:8.2f} {self.TTC[i]:8.2f}')
        '''

        # Publish to drive if a TTC is too low
        # min(self.TTC) < self.velocity_x / breaking_deceleration
        if min(self.TTC) < max(TTB, self.velocity_x / breaking_deceleration):
            # Initialize message
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0

            # Publish message to topic (which is initialized in the publisher itself)
            self.drive_pub.publish(msg)
            self.get_logger().info(f"EMERGENCY BREAKING: TTC {min(self.TTC):.5f}")

def main():
    rclpy.init() 
    node = EmergencyBreak() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        