import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')  
        
        # Create a subscriber, type: Acker, topic 'drive', function to run when receiving messages, and queue size
        self.sub = self.create_subscription(AckermannDriveStamped, "drive", self.drive_callback, 1) 

        # Create a new publisher to 'drive_relay'
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive_relay', 5)

    #Callback function: modify and print the received information
    def drive_callback(self, msg):
        # Initialize new Ack message
        new_msg = AckermannDriveStamped()

        # Multiply speed and angle by 3
        new_msg.drive.speed = msg.drive.speed * 3.0
        new_msg.drive.steering_angle = msg.drive.steering_angle * 3.0

        # Publish to drive_relay
        self.pub.publish(new_msg)

def main():
    rclpy.init() 
    node = Relay() 
    rclpy.spin(node)
    node.destroy_node()  
    rclpy.shutdown()        