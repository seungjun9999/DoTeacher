#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    share_dir = get_package_share_directory('custom_slam')
    parameter_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rvizconfig')
    node_name = 'slam_toolbox'

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'mapper_params_online_sync.yaml'),
        description='Full path to the ROS2 parameters file to use.'
    )

    rviz_declare = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(share_dir, 'config', 'custom_slam_rviz.rviz'),
        description='Full path to the RViz config file.'
    )

    driver_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        remappings=[('/scan', 'scan')],
        arguments=['--ros-args', '--param', 'qos_overrides./scan.publisher.reliability:=best_effort']
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        params_declare,
        rviz_declare,
        driver_node,
        tf2_node,
        rviz_node
    ])
