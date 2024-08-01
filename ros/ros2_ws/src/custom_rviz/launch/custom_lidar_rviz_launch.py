from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description(): # launch default
    share_dir = get_package_share_directory('ydlidar_ros2_driver') # create share directory
    rviz_config_file = os.path.join(share_dir, 'config', 'custom.rviz') # config path
    parameter_file = LaunchConfiguration('params_file') #  use param
    node_name = 'custom_ros2_ydlidar_node' # node name

    # param path
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'X4-Pro.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    # ydlidar ros2 driver node
    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file], # set param
        namespace='/'
    )

    # tf node
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
    )

    # rviz node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        rviz2_node,
    ])