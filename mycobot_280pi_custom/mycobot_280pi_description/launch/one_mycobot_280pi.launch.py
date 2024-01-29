#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot_name = LaunchConfiguration("robot_name")
    robot_xyz = LaunchConfiguration("robot_xyz")
    robot_rpy = LaunchConfiguration("robot_rpy")

    description_package = FindPackageShare('mycobot_280pi_description')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", 'mycobot_280pi.urdf.xacro']),
            " ",
            "robot_name:=",
            robot_name.perform(context) + "_",
            " ",
            "origin_xyz:=",
            robot_xyz,
            " ",
            "origin_rpy:=",
            robot_rpy,
        ]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output="screen"
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "mycobot_280pi.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    # nodes = [robot_state_publisher_node, joint_state_publisher_node, rviz_node]
    nodes = [robot_state_publisher_node, rviz_node]
    return nodes

def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="robot0"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_xyz",
            default_value='"0 0 0"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_rpy",
            default_value='"0 0 0"',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
