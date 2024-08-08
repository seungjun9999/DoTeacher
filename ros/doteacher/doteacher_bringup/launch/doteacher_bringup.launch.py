# launch/doteacher_bring_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('doteacher_bring')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    param_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(bringup_dir, 'config', 'map.yaml')
    ydlidar_config = os.path.join(bringup_dir, 'config', 'ydlidar.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'map': LaunchConfiguration('map')
            }.items(),
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[ydlidar_config]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        Node(
            package='doteacher_teleops',
            executable='teleop_node',
            name='teleop_joy_node',
            output='screen'),

        Node(
            package='doteacher_control',
            executable='motor_controller',
            name='motor_controller_node',
            output='screen'),
    ])
