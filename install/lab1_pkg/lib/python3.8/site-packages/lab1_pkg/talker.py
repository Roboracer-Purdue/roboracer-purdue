import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare parameters
        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        # Create a publisher for the ackermann message (Type of Message, Topic Name, Queue Size)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 1)

        # Create a timer that call a function to publish to drive topic
        self.timer = self.create_timer(0.1, self.publish_to_drive)

    def publish_to_drive(self):
        # Get the parameters
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        # Initial message
        msg = AckermannDriveStamped()
        msg.drive.speed = v
        msg.drive.steering_angle = d

        # Publish message to topic (which is initialized in the publisher itself)
        self.drive_publisher.publish(msg)
    
def main(args=None):
    # Start rclpy
    rclpy.init(args=args)

    # Starting the node
    node = Talker()
    rclpy.spin(node)

    # Stopping the processes when the spin stop (when the program is terminated)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()