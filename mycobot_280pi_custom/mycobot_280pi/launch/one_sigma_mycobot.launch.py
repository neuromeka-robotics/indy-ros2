#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    robot_name = LaunchConfiguration("robot_name")
    sigma_name = LaunchConfiguration("sigma_name")

    sigma_mycobot_0 = Node(
        package="mycobot_280pi",
        executable="sigma_mycobot.py",
        name="sigma_mycobot",
        output="screen",
        parameters=[
            {'robot': robot_name.perform(context)},
            {'sigma': sigma_name.perform(context)},
        ],
    )

    return [sigma_mycobot_0,]


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
            "sigma_name",
            default_value="sigma0",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
