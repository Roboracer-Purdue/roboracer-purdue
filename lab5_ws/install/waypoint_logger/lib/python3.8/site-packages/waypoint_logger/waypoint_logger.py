import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion

home = expanduser('~')
file = open(strftime(home+'/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')

def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)

    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

class waypoint_logger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')  
        
        # Declare Paramters
        self.declare_parameter('minL', 0.1)

        # Initialize Node Variables
        self.posePositionX = -999
        self.posePositionY = -999

        # Subscribe to odom to update car velocity
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 5) 

    def odom_callback(self, msg):
        look_ahead = self.get_parameter('minL').get_parameter_value().double_value
        look_ahead **= 2
        # Log Waypoint if It's far enough from OG
        if (msg.pose.pose.position.x - self.posePositionX) ** 2 + (msg.pose.pose.position.y - self.posePositionY) ** 2 > look_ahead:  
            save_waypoint(msg)
            self.posePositionX = msg.pose.pose.position.x
            self.posePositionY = msg.pose.pose.position.y

def main():
    rclpy.init() 
    node = waypoint_logger() 
    rclpy.spin(node)

    # Close file and Stop Node
    file.close()
    node.destroy_node()  
    rclpy.shutdown()        