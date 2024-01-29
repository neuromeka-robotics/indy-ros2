#!/usr/bin/python3
#-*- coding: utf-8 -*-

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('mycobot_280pi_description')
    driver_package = FindPackageShare('mycobot_280pi')

    # Initialize Arguments
    robot_name = LaunchConfiguration("robot_name")
    robot_xyz = LaunchConfiguration("robot_xyz")
    robot_rpy = LaunchConfiguration("robot_rpy")

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
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [driver_package, "controller", "mycobot_controller.yaml"]
    )
    modified_initial_joint_controllers = PathJoinSubstitution(
        [driver_package, "controller", robot_name.perform(context) + "_mycobot_controller.yaml"]
    )

    # Load the initial_joint_controllers YAML file
    initial_joint_controllers_file = initial_joint_controllers.perform(context)
    with open(initial_joint_controllers_file, "r") as file:
        initial_joint_controllers_yaml = yaml.safe_load(file)

    if "joint_trajectory_controller" in initial_joint_controllers_yaml:
        controller_params = initial_joint_controllers_yaml["joint_trajectory_controller"]
        if "ros__parameters" in controller_params:
            ros_params = controller_params["ros__parameters"]
            if "joints" in ros_params:
                modified_joints = [robot_name.perform(context) + "_" + joint for joint in ros_params["joints"]]
                ros_params["joints"] = modified_joints

    modified_initial_joint_controllers_file = modified_initial_joint_controllers.perform(context)
    with open(modified_initial_joint_controllers_file, "w") as file:
        yaml.dump(initial_joint_controllers_yaml, file)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    mycobot_interface = Node(
        package="mycobot_280pi",
        executable="mycobot_interface.py",
        name="mycobot_interface",
        output="screen",
        parameters=[
            {'robot': robot_name.perform(context)},
        ],
    )

    nodes_to_start = [
        mycobot_interface,
        robot_state_publisher_node,
    ]

    return nodes_to_start


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
