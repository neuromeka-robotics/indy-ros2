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

    config_file = LaunchConfiguration("config_file").perform(context)

    config_dict = load_yaml(os.path.join(__driver_dir, "config", config_file))
    robot_names = config_dict["robot_names"]

    robot_config_dict = {}

    for i, robot_name in enumerate(robot_names):
        robot_config = config_dict[robot_name]
        robot_config["robot_name"] = robot_name
        robot_config_dict[robot_name] = robot_config

    mycobot_drivers = []
    for robot_key, robot_conf in robot_config_dict.items():
        mycobot_driver = Node(
            package='indy_driver',
            executable='mycobot_driver.py',
            output='screen',
            parameters=[robot_conf],
        )
        mycobot_drivers.append(mycobot_driver)

    return mycobot_drivers


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="1robot_config.yaml",
            description="Robot configurations file name"
        ),
        OpaqueFunction(function=launch_setup)])
