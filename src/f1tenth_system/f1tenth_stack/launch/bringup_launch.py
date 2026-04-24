#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')

    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    throttle_mix_yaml = os.path.join(pkg_share, 'config', 'throttle_mix.yaml')

    joy_la = DeclareLaunchArgument('joy_config', default_value=joy_teleop_config)
    vesc_la = DeclareLaunchArgument('vesc_config', default_value=vesc_config)
    sensors_la = DeclareLaunchArgument('sensors_config', default_value=sensors_config)
    mux_la = DeclareLaunchArgument('mux_config', default_value=mux_config)
    throttle_mix_la = DeclareLaunchArgument('throttle_mix_config', default_value=throttle_mix_yaml)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
    )

    throttle_mix_node = Node(
        package='f1tenth_stack',
        executable='throttle_mix',
        name='throttle_mix',
        output='screen',
        parameters=[LaunchConfiguration('throttle_mix_config')],
    )

    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
    )

    hokuyo_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='hokuyo',
        output='screen',
        parameters=[LaunchConfiguration('sensors_config')],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        joy_la,
        vesc_la,
        sensors_la,
        mux_la,
        throttle_mix_la,

        joy_node,
        joy_teleop_node,
        throttle_mix_node,
        ackermann_mux_node,
        ackermann_to_vesc_node,
        vesc_to_odom_node,
        vesc_driver_node,
        hokuyo_node,
        static_tf_node,
    ])