from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# append
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    
    package_name='doteacher'
    
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    # )

    # rviz_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare("doteacher"),
    #         "config/rviz",
    #         "launch_sim.rviz",
    #     ]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )

    # robot_description_path = os.path.join(get_package_share_directory('doteacher'),
    #                                       'urdf', 'doteacher.urdf.xacro')
    # robot_description_config = xacro.process_file(robot_description_path)
    # robot_description = {'robot_description': robot_description_config.toxml()}

    
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("doteacher"),
    #         "config", "ros2_control",
    #         "doteacher_controllers.yaml",
    #     ]
    # )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])


    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','ros2_control','doteacher_controllers.yaml')


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
    )

    delayed_ackermann_steering_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_steering_spawner],
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    
    nodes = [
        rsp,
        delayed_controller_manager,
        delayed_ackermann_steering_spawner,
        delayed_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
