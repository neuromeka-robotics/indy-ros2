#!/usr/bin/python3
#-*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    driver_package = FindPackageShare('mycobot_280pi')
    moveit_config_package = FindPackageShare('mycobot_280pi_moveit')

    # Initialize Arguments
    robot_name = LaunchConfiguration("robot_name")
    robot_xyz = LaunchConfiguration("robot_xyz")
    robot_rpy = LaunchConfiguration("robot_rpy")

    mycobot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [driver_package, "/launch", "/mycobot_interface.launch.py"]
        ),
        launch_arguments={
            "robot_name": robot_name,
            "robot_xyz": robot_xyz,
            "robot_rpy": robot_rpy,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "robot_name": robot_name,
            "robot_xyz": robot_xyz,
            "robot_rpy": robot_rpy,
        }.items(),
    )

    nodes_to_launch = [
        mycobot_bringup_launch,
        moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
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