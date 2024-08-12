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


def generate_launch_description():
    
    package_name='doteacher'
    
    # gui = LaunchConfiguration("gui")

    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/demo/cmd_demo')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo','gazebo_params.yaml')

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
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'doteacher'],
        output='screen'
    )
    
    # delayed_spawn_entity = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=gazebo,
    #         on_start=[spawn_entity],
    #     )
    # )

    ackermann_steering_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
    )
    
    delayed_ackermann_steering_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
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
            target_action=spawn_entity,
            on_start=[joint_state_broadcaster_spawner],
        )
    )
    
    nodes = [
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        # ackermann_steering_spawner,
        # joint_state_broadcaster_spawner,
        # rviz_node,
        # delayed_spawn_entity,
        delayed_ackermann_steering_spawner,
        delayed_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)
