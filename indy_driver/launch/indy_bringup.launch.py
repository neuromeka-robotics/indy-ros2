from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction , RegisterEventHandler
# from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import sys
import os
__driver_dir = get_package_share_directory('indy_driver')
sys.path.append(os.path.join(__driver_dir, "nrmk_utils"))
from file_io import *

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')

    config_file = LaunchConfiguration("config_file").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")

    config_dict = load_yaml(os.path.join(__driver_dir, "config", config_file))
    desc_file_name = config_dict["desc_file_name"]
    robot_names = config_dict["robot_names"]

    robot_config_dict = {}
    robot_description_parts = []

    for i, robot_name in enumerate(robot_names):
        robot_config = config_dict[robot_name]
        robot_config["robot_name"] = robot_name
        robot_config_dict[robot_name] = robot_config

        robot_description_parts.extend(
            [
                f' robot{i}_name:={robot_name}_',
                f' origin{i}_xyz:="{" ".join(map(str, robot_config["origin"]["xyz"]))}"',
                f' origin{i}_rpy:="{" ".join(map(str, robot_config["origin"]["rpy"]))}"',
                f' indy{i}_type:={robot_config["robot_type"]}',
                f' indy{i}_eye:={str(robot_config["indy_eye"]).lower()}',
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

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "indy.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    indy_drivers = []
    for robot_key, robot_conf in robot_config_dict.items():
        indy_driver = Node(
            package='indy_driver',
            executable='indy_driver.py',
            output='screen',
            parameters=[robot_conf],
        )
        indy_drivers.append(indy_driver)

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node
    ]

    return nodes_to_start + indy_drivers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="1robot_config.yaml",
            description="Robot configurations file name"
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?"
        ),
        OpaqueFunction(function=launch_setup)])
