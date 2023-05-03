# import os
from launch import LaunchDescription

from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer

from indy_moveit.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')
    moveit_config_package = FindPackageShare('indy_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    prefix = LaunchConfiguration("prefix")
    launch_rviz_servo = LaunchConfiguration("launch_rviz_servo")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", "indy.urdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "indy_type:=",
            indy_type,
            " ",
            "indy_eye:=",
            indy_eye,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_config_package, "srdf", "indy.srdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "indy_type:=",
            indy_type,
            " ",
            "indy_eye:=",
            indy_eye,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [moveit_config_package, "rviz_config", "indy_servo.rviz"]
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz_servo),
        package="rviz2",
        executable="rviz2",
        name="rviz2_servo",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("indy_moveit", "moveit_config/indy_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # ComposableNode(
            #     package='robot_state_publisher',
            #     plugin='robot_state_publisher::RobotStatePublisher',
            #     name='robot_state_publisher',
            #     parameters=[robot_description],
            # ),
            # ComposableNode(
            #     package='tf2_ros',
            #     plugin='tf2_ros::StaticTransformBroadcasterNode',
            #     name='static_tf2_broadcaster',
            #     parameters=[{'child_frame_id': 'link0', 'frame_id': 'world'}],
            # ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoServer",
                name="servo_server",
                parameters=[
                    servo_params,
                    robot_description,
                    robot_description_semantic,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    nodes_to_start = [
        container,
        rviz_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2", "indy12", "indyrp2", "indyrp2_v2"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz_servo", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
