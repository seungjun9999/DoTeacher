from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['ackermann_steering_controller'],
            output='screen',
        ),
    ])
