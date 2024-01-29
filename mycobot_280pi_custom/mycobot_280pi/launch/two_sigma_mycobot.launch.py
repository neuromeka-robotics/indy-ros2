#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot0_name = LaunchConfiguration("robot0_name")
    sigma0_name = LaunchConfiguration("sigma0_name")
    robot1_name = LaunchConfiguration("robot1_name")
    sigma1_name = LaunchConfiguration("sigma1_name")

    sigma_mycobot_0 = Node(
        package="mycobot_280pi",
        executable="sigma_mycobot.py",
        name="sigma_mycobot",
        output="screen",
        parameters=[
            {'robot': robot0_name.perform(context)},
            {'sigma': sigma0_name.perform(context)},
        ],
    )

    sigma_mycobot_1 = Node(
        package="mycobot_280pi",
        executable="sigma_mycobot.py",
        name="sigma_mycobot",
        output="screen",
        parameters=[
            {'robot': robot1_name.perform(context)},
            {'sigma': sigma1_name.perform(context)},
        ],
    )

    nodes = [
        sigma_mycobot_0,
        sigma_mycobot_1
    ]

    return nodes


def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot0_name",
            default_value="robot0"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sigma0_name",
            default_value="sigma0",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_name",
            default_value="robot1"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sigma1_name",
            default_value="sigma1",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

