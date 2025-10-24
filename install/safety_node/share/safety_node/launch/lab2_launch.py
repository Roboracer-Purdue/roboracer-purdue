from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_node',
            executable='emergency_break',
            name='emergency_break',
            parameters=[{
                'breaking_deceleration': 15.0,
                'barrier_width': 0.5,
                'TTB': 0.4
            }]
        ),
    ])