from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    wall_follow_node = Node(
        package='wall_follow_2',      
        executable='wall_follow',      
        name='wall_follow',
        output='screen',
        parameters=[
            {
                # Topics
                'scan_topic': '/scan',
                'drive_topic': '/drive',
                'odom_topic': '/ego_racecar/odom',

                # LiDAR processing
                'front_fov_deg': 200.0,
                'range_clip_max': 10.0,
                'range_clip_min': 0.05,

                # Steering
                'steer_limit_rad': 0.40,
                'steer_smooth_alpha': 0.65,

                # Speed
                'speed_min': 0.5,
                'speed_max': 2.0,

                # Wall following
                'target_distance': 1.1,
                'look_ahead': 1.0,
                'beam_a_angle': -15.0, #340
                'beam_b_angle': -90.0, #179

                'K_p': 1.5, 
                'K_i': 0.0,
                'K_d': 3.6,

                # Safety
                'ttc_emergency': 0.5,
                'ttc_slow': 0.7,

            }
        ]
    )

    return LaunchDescription([
        wall_follow_node
    ])