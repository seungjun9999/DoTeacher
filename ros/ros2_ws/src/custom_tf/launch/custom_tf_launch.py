#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share_dir = get_package_share_directory('custom_tf')
    parameter_file = LaunchConfiguration('params_file')

    # params_declare = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(share_dir, 'config', 'mapper_params_online_sync.yaml'),
    #     description='Full path to the ROS2 parameters file to use.'
    # )

    odom_tf2_node = Node(
        package='custom_tf',
        executable='custom_tf_node',
        name='custom_tf_node',
        output='screen',
    ),


    slam_tf2_node = Node(
        package='custom_tf',
        executable='custom_tf_slam_node',
        name='custom_tf_slam_node',
        output='screen',
    ),

    return LaunchDescription([
        # params_declare,
        odom_tf2_node,
        slam_tf2_node
    ])
