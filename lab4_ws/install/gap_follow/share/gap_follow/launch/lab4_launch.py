from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gap_follow',
            executable='gap_follow',
            name='gap_follow',
            parameters=[{
                'target_distance': 1.0,
                'look_ahead': 1.0,
                'beam_a_id': 99.0, 
                'beam_b_id': 981.0,
                'turn_a_id': 420.0, 
                'turn_b_id': 1080.0,
                'bubble_radius': 1.0, # Original 0.5 0.2 1.0
                'disparity_thresh': 0.5,
            }]
        ),
    ])