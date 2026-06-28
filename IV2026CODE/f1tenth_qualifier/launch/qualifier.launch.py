from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="f1tenth_qualifier",
            executable="qualifier_node",
            name="qualifier_node",
            output="screen",
            parameters=[
                "/home/arc/f1tenth_ws/src/f1tenth_qualifier/config/qualifier.yaml"
            ],
        )
    ])