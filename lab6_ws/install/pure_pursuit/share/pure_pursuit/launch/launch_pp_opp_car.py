from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[{
                'odom_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
                'scan_topic': '/opp_scan',
                'max_velocity': 4.0,
                'use_rrt': False
            }]
        ),


    ])