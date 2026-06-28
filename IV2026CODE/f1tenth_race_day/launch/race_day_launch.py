from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("f1tenth_race_day"),
        "config",
        "race_day_params.yaml",
    ])

    race_day_node = Node(
        package="f1tenth_race_day",
        executable="race_day_node",
        name="f1tenth_race_day",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([
        race_day_node,
    ])