#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')

    # Initialize Arguments
    robot_quantity = LaunchConfiguration("robot_quantity")

    desc_file_name = "one_indy.urdf.xacro"
    if int(robot_quantity.perform(context)) == 2:
        desc_file_name = "two_indy.urdf.xacro"
    elif int(robot_quantity.perform(context)) == 4:
        desc_file_name = "four_indy.urdf.xacro"

    robot_description_parts = []
    for i in range(int(robot_quantity.perform(context))):

        robot_name = LaunchConfiguration(f"robot{i}_name")
        robot_xyz = LaunchConfiguration(f"robot{i}_xyz")
        robot_rpy = LaunchConfiguration(f"robot{i}_rpy")
        indy_type = LaunchConfiguration(f"indy{i}_type")
        indy_eye = LaunchConfiguration(f"indy{i}_eye")

        robot_description_parts.extend(
            [
                f" robot{i}_name:={robot_name.perform(context)}_",
                f" origin{i}_xyz:={robot_xyz.perform(context)}",
                f" origin{i}_rpy:={robot_rpy.perform(context)}",
                f" indy{i}_type:={indy_type.perform(context)}",
                f" indy{i}_eye:={indy_eye.perform(context)}",
            ]
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", desc_file_name]),
            " ",
        ] + robot_description_parts
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "indy.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    nodes = [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return nodes

def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    robots = [
        {"name": "robot0", "xyz": '"0 0 0"', "rpy": '"0 0 0"', "indy_type": "indy12", "indy_eye": "false"},
        {"name": "robot1", "xyz": '"1 0 0"', "rpy": '"0 0 0"', "indy_type": "indy7", "indy_eye": "true"},
        {"name": "robot2", "xyz": '"1 1 0"', "rpy": '"0 0 0"', "indy_type": "indy7_v2", "indy_eye": "false"},
        {"name": "robot3", "xyz": '"1 0 0"', "rpy": '"0 0 0"', "indy_type": "indyrp2", "indy_eye": "true"}
    ]

    for i, robot in enumerate(robots):
        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_name",
                default_value=robot['name']
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_xyz",
                default_value=robot['xyz'],
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_rpy",
                default_value=robot['rpy'],
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"indy{i}_type",
                default_value=robot['indy_type'],
                description="Type of Indy robot.",
                choices=["indy7", "indy7_v2", "indy12", "indy12_v2", "indyrp2", "indyrp2_v2"]
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"indy{i}_eye",
                default_value=robot['indy_eye'],
                description="Work with Indy Eye",
            )
        )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_quantity",
            default_value="1",
            description="Robot quantity. Valid number are 1, 2 and 4",
            choices=["1", "2", "4"]
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
