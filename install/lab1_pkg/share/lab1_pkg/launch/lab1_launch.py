from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            parameters=[{
                'v': 6.9,
                'd': 4.2
            }]
        ),
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay'
        )
    ])