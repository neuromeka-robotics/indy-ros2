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
    robot0_name = LaunchConfiguration("robot0_name")
    robot0_xyz = LaunchConfiguration("robot0_xyz")
    robot0_rpy = LaunchConfiguration("robot0_rpy")
    robot1_name = LaunchConfiguration("robot1_name")
    robot1_xyz = LaunchConfiguration("robot1_xyz")
    robot1_rpy = LaunchConfiguration("robot1_rpy")

    mycobot0_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [driver_package, "/launch", "/mycobot_interface.launch.py"]
        ),
        launch_arguments={
            "robot_name": robot0_name,
            "robot_xyz": robot0_xyz,
            "robot_rpy": robot0_rpy,
        }.items(),
    )

    mycobot1_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [driver_package, "/launch", "/mycobot_interface.launch.py"]
        ),
        launch_arguments={
            "robot_name": robot1_name,
            "robot_xyz": robot1_xyz,
            "robot_rpy": robot1_rpy,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/two_robot_moveit.launch.py"]
        ),
        launch_arguments={
            "robot0_name": robot0_name,
            "robot0_xyz": robot0_xyz,
            "robot0_rpy": robot0_rpy,
            "robot1_name": robot1_name,
            "robot1_xyz": robot1_xyz,
            "robot1_rpy": robot1_rpy,
        }.items(),
    )

    nodes_to_launch = [
        mycobot0_bringup_launch,
        mycobot1_bringup_launch,
        moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot0_name",
            default_value="robot0"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot0_xyz",
            default_value='"0 0 0"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot0_rpy",
            default_value='"0 0 0"',
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
            "robot1_xyz",
            default_value='"0 0.5 0"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot1_rpy",
            default_value='"0 0 0"',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
