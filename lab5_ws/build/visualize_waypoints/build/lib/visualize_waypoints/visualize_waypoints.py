import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from os.path import expanduser
import csv

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

home = expanduser('~')

class visualize_waypoints(Node):
    def __init__(self):
        super().__init__('visualize_waypoints')  
        
        # Declare Paramters
        self.declare_parameter('filename', "levine_manual_waypoints_1.csv")

        self.publisher = self.create_publisher(MarkerArray, '/waypoints', 10)

        filename = self.get_parameter("filename").value
        filename = home + '/' + filename
        
        with open(filename, "r") as file:
            data = csv.reader(file)

            marker_array = MarkerArray()

            j = 0

            for i in data:
                x, y = float(i[0]), float(i[1])

                marker = Marker()

                marker.id = j
                j += 1

                marker.header.frame_id = "map"
                marker.ns = "points"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                self.get_logger().info(f"Waypoint at ({x:.3f}, {y:.3f})")
                marker_array.markers.append(marker)

            self.publisher.publish(marker_array)
            

def main():
    rclpy.init() 
    node = visualize_waypoints() 
    rclpy.spin(node)

    # Close file and Stop Node
    file.close()
    node.destroy_node()  
    rclpy.shutdown()        