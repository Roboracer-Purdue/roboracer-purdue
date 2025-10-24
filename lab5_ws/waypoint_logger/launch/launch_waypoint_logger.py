from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='waypoint_logger',
            executable='waypoint_logger',
            name='waypoint_logger',
            parameters=[{
                'minL': 0.1,
            }]
        ),
    ])