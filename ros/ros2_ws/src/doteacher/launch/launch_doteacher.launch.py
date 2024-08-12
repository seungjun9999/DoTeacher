from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )


    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("doteacher"), "urdf", "doteacher.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("doteacher"),
            "config/ros2_control",
            "doteacher_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("doteacher"),
            "config/rviz",
            "doteacher.rviz",
        ]
    )

    twist_mux_file = PathJoinSubstitution(
        [
            FindPackageShare("doteacher"),
            "config",
            "twist_mux.yaml",
        ]
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_file],
        remappings=[('/cmd_vel_out','/cmd_vel_mux')]
    )

    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        remappings=[
            ("/cmd_vel_in", "/cmd_vel_mux"),
            ("/cmd_vel_out", "/bicycle_steering_controller/reference"),
        ],
    )

    # the steering controller libraries by default publish odometry on a separate topic than /tf
    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/bicycle_steering_controller/tf_odometry", "/tf"),
            # ("/bicycle_steering_controller/odometry", "/odom"),
        ],
        condition=IfCondition(remap_odometry_tf),
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(remap_odometry_tf),
    )
    robot_state_pub_bicycle_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_bicycle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bicycle_steering_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_bicycle_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        control_node_remapped,
        twist_mux,
        twist_stamper_node,
        robot_state_pub_bicycle_node,
        robot_bicycle_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
