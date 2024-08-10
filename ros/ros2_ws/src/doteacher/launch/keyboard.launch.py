from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    teleop_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard', '--ros-arg', '/cmd_vel:=/cmd_vel_keyboard'],
        name='teleop_keyboard_node',
        output='screen'
    )

    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ("/cmd_vel_in", "/cmd_vel_keyboard"),
            ("/cmd_vel_out", "/bicycle_steering_controller/reference"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        teleop_node,
        twist_stamper_node       
    ])