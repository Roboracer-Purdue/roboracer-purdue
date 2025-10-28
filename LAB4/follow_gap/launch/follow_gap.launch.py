from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_gap',
            executable='follow_gap',
            name='follow_gap_node',
            output='screen'
        )
    ])
