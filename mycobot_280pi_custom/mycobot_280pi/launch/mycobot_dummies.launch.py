#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    number_of_robot = LaunchConfiguration("quantity")

    robots = []
    for i in range (int(number_of_robot.perform(context))):
        robots.append(
            Node(
                package="mycobot_280pi",
                executable="control_node_dummy.py",
                name="control_node_dummy",
                output="screen",
                parameters=[
                    {'robot': "robot" + str(i)},
                ],
            )
        )

    return robots


def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "quantity",
            default_value="1"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

