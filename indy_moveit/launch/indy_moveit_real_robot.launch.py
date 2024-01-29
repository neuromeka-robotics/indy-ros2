from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import sys
import os
__driver_dir = get_package_share_directory('indy_driver')
sys.path.append(os.path.join(__driver_dir, "nrmk_utils"))
from file_io import *

def launch_setup(context, *args, **kwargs):
    
    indy_driver_package = FindPackageShare('indy_driver')
    moveit_config_package = FindPackageShare('indy_moveit')

    config_file = LaunchConfiguration("config_file").perform(context)

    config_dict = load_yaml(os.path.join(__driver_dir, "config", config_file))
    desc_file_name = config_dict["desc_file_name"]
    robot_names = config_dict["robot_names"]

    robot_config_dict = {}
    moveit_launch_arguments = {}

    for i, robot_name in enumerate(robot_names):
        robot_config = config_dict[robot_name]
        robot_config["robot_name"] = robot_name
        robot_config_dict[robot_name] = robot_config

        moveit_launch_arguments.update(
            {
                f" robot{i}_name": robot_name,
                f" origin{i}_xyz": f'"{" ".join(map(str, robot_config["origin"]["xyz"]))}"',
                f" origin{i}_rpy": f'"{" ".join(map(str, robot_config["origin"]["rpy"]))}"',
                f" indy{i}_type": robot_config["robot_type"],
                f" indy{i}_eye": str(robot_config["indy_eye"]).lower()
            }
        )


    bringup_launch_arguments = {
        "config_file": config_file,
        "launch_rviz": "false"
    }

    moveit_launch_arguments.update(
        {
            "robot_quantity": str(len(robot_names)),
            "launch_rviz_moveit": "true",
        }
    )

    indy_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [indy_driver_package, "/launch", "/indy_bringup.launch.py"]
        ),
        launch_arguments=bringup_launch_arguments.items(),
    )

    indy_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments=moveit_launch_arguments.items(),
    )

    nodes_to_launch = [
        indy_bringup_launch,
        indy_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="1robot_config.yaml",
            description="Robot configurations file name"
        ),
        OpaqueFunction(function=launch_setup)])
