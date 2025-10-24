import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

num_beam = 1080

class PID(Node):
    def __init__(self):
        super().__init__('PID')  
        
        # Declare Paramters
        self.declare_parameter('target_distance', 0.0)
        self.declare_parameter('look_ahead', 0.0)
        self.declare_parameter('beam_a_id', 0.0)
        self.declare_parameter('beam_b_id', 0.0)
        self.declare_parameter('K_p', 0.0)
        self.declare_parameter('K_i', 0.0)
        self.declare_parameter('K_d', 0.0)
        

        # Initialize Node Variables
        self.velocity_x = 1.0
        self.max_vel = 1.0
        self.prev_steering = 0.0
        self.prev_error = 0.0

        # Create subscribers, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1) 

        # Subscribe to odom to update car velocity
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 5) 

        # Create a new publisher to 'drive_relay'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 5)

        # Create a timer to call a function that check for potential collsion
        # None

    def odom_callback(self, msg):
        # RETRIEVE sth from odom message
        self.velocity_x = msg.twist.twist.linear.x
        
    def scan_callback(self, msg):
        # Retrieve Parameters
        target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        look_ahead = self.get_parameter('look_ahead').get_parameter_value().double_value
        beam_a_id = int(self.get_parameter('beam_a_id').get_parameter_value().double_value)
        beam_b_id = int(self.get_parameter('beam_b_id').get_parameter_value().double_value)
        K_p = self.get_parameter('K_p').get_parameter_value().double_value
        K_i = self.get_parameter('K_i').get_parameter_value().double_value
        K_d = self.get_parameter('K_d').get_parameter_value().double_value
        """
        self.get_logger().info(f"target_distance: {target_distance:.3f}")
        self.get_logger().info(f"look_ahead: {look_ahead:.3f}")
        self.get_logger().info(f"beam_a_id: {beam_a_id:.3f}")
        self.get_logger().info(f"beam_b_id: {beam_b_id:.3f}")
        self.get_logger().info(f"K PID: {K_p:.3f} | {K_i:.3f} | {K_d:.3f}")
        """


        # Retrive a and b laser scan
        a = msg.ranges[beam_a_id]
        b = msg.ranges[beam_b_id]
        a_angle = msg.angle_min + msg.angle_increment * beam_a_id
        b_angle = msg.angle_min + msg.angle_increment * beam_b_id
        theta = b_angle - a_angle
        #self.get_logger().info(f'a_angle: {a_angle:.3f} b_angle: {b_angle:.3f} theta: {theta:.3f}')

        # Calculate alpha
        alpha = np.arctan((a * np.cos(theta) - b)/(a * np.sin(theta)))
        #self.get_logger().info(f'Alpha: {alpha:.3f}')

        # Caculate Dt and Dt1
        Dt = b * np.cos(alpha)
        Dt1 = Dt + look_ahead * np.sin(alpha)
        #self.get_logger().info(f'Dt: {Dt:.3f} Dt1: {Dt1:.3f} ')

        # Compute PID steering angles
        et = target_distance - Dt
        et1 = target_distance - Dt1

        t = look_ahead / max(self.velocity_x, 0.00001)

        term1 = et1
        term2 = (et1 + et) / 2 * t
        term3 = (et - et1)

        ut = (term1 * K_p + term2 * K_i + term3 * K_d)
        self.max_vel = max(self.velocity_x, self.max_vel)

        self.prev_steering = ut
        self.prev_error = et1
        
        # self.get_logger().info(f'{ut:.3f}')
        self.get_logger().info(f'{ut:.3f} | {term1:.3f} {term2:.3f} {term3:.3f}')

        # Calculate driving speed
        if msg.ranges[539] > 5 and msg.ranges[541] > 5 and msg.ranges[541] > 5 and abs(ut) < 0.1:
            speed = 2.0
        elif abs(ut) < 5:
            speed = 1.5
        elif abs(ut) < 10:
            speed = 1.5
        elif abs(ut) < 20:
            speed = 1.0
        else:
            speed = 0.5
        
        # Publish to drive
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = ut

        #msg.drive.speed = 0.0
        #msg.drive.steering_angle = 0.0
    
        # Publish message to topic (which is initialized in the publisher itself)
        self.drive_pub.publish(msg)

def main():
    rclpy.init() 
    node = PID() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        