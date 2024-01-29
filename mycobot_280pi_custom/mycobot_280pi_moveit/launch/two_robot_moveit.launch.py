#!/usr/bin/python3
#-*- coding: utf-8 -*-

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from mycobot_280pi_moveit.launch_common import load_yaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('mycobot_280pi_description')
    moveit_config_package = FindPackageShare('mycobot_280pi_moveit')

    # Initialize Arguments    
    robot0_name = LaunchConfiguration("robot0_name")
    robot0_xyz = LaunchConfiguration("robot0_xyz")
    robot0_rpy = LaunchConfiguration("robot0_rpy")
    robot1_name = LaunchConfiguration("robot1_name")
    robot1_xyz = LaunchConfiguration("robot1_xyz")
    robot1_rpy = LaunchConfiguration("robot1_rpy")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", 'two_mycobot_280pi.urdf.xacro']),
            " ",
            "robot0_name:=",
            robot0_name.perform(context) + "_",
            " ",
            "origin0_xyz:=",
            robot0_xyz,
            " ",
            "origin0_rpy:=",
            robot0_rpy,
            " ",
            "robot1_name:=",
            robot1_name.perform(context) + "_",
            " ",
            "origin1_xyz:=",
            robot1_xyz,
            " ",
            "origin1_rpy:=",
            robot1_rpy,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_config_package, "srdf", 'two_mycobot_280pi.srdf.xacro']),
            " ",
            "robot0_name:=",
            robot0_name.perform(context) + "_",
            " ",
            "robot1_name:=",
            robot1_name.perform(context) + "_",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [moveit_config_package, "moveit_config", "two_robot_kinematics.yaml"]
    )
    
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("mycobot_280pi_moveit", "moveit_config/two_robot_ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("mycobot_280pi_moveit", "moveit_config/two_robot_controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
        "trajectory_execution.trajectory_duration_monitoring": False
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [moveit_config_package, "rviz_config", "mycobot_280pi_moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
        ],
    )

    nodes_to_start = [move_group_node, rviz_node]
    return nodes_to_start
    # return [move_group_node]

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
