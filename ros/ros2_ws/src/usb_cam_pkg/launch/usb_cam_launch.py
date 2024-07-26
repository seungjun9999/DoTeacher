import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Create the display_node first
    display_node = Node(
        package='usb_cam_pkg',
        executable='display_node',
        name='display_node',
        output='screen'
    )

    detect_marker_node = Node(
        package='usb_cam_pkg',
        executable='detect_marker_node',
        name='detect_marker_node',
        output='screen'
    )

    detect_pose_node = Node(
        package='usb_cam_pkg',
        executable='detect_pose_node',
        name='detect_pose_node',
        output='screen'
    )

    # Then create the usbcam_node after the display_node
    usbcam_node = Node(
        package='usb_cam_pkg',
        executable='usbcam_node',
        name='usbcam_node',
        output='screen'
    )

    return LaunchDescription([
        display_node,
        detect_marker_node,
        detect_pose_node,
        usbcam_node
    ])
