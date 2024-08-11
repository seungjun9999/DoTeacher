from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# append
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    package_name='doteacher'
    
    gui = LaunchConfiguration("gui")

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

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
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'doteacher'],
        output='screen'
    )

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    
    nodes = [
        rsp,
        gazebo,
        spawn_entity,
        ackermann_steering_spawner,
        joint_state_broadcaster_spawner,
        # rviz_node,
    ]

    return LaunchDescription(nodes)
