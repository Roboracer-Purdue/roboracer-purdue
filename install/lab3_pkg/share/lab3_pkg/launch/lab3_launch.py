from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab3_pkg',
            executable='PID',
            name='PID',
            parameters=[{
                'target_distance': 1.0,
                'look_ahead': 1.0,
                'beam_a_id': 400.0, # Original = 340.0
                'beam_b_id': 179.0,
                'K_p': 0.5, # Original 0.5 0.2 1.0
                'K_i': 0.2,
                'K_d': 1.0,
            }]
        ),
    ])