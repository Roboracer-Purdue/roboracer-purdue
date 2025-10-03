from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='break_tester',
            executable='break_test',
            name='break_test',
        ),
    ])