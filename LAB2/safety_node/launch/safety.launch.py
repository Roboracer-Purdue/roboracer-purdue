from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_node',
            executable='aeb_node',
            name='aeb_node',
            output='screen',
            parameters=[{
                # You can tune these at launch time
                'ittc_threshold': 0.5,       # seconds
                'min_range': 0.10,           # meters (filter noise)
                'fov_deg': 70.0,            # forward arc to consider
                'use_diff_range_rate': False # alternative range-rate method
            }]
        )
    ])
