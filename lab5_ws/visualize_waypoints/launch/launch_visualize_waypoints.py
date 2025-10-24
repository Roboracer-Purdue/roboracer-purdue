from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visualize_waypoints',
            executable='visualize_waypoints',
            name='visualize_waypoints',
            parameters=[{
                'filename': "levine_manual_waypoints_1.csv",
            }]
        ),
    ])