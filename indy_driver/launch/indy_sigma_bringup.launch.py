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
    config0 = LaunchConfiguration("config0")
    config_dicts = [load_yaml(os.path.join(__driver_dir, "config", config0.perform(context)))]
    config1 = LaunchConfiguration("config1")
    cname1 = config1.perform(context)
    if cname1 != "":
        config_dicts.append(load_yaml(os.path.join(__driver_dir, "config", cname1)))

    return [
        Node(
            package='indy_driver',
            executable='indy_sigma.py',
            output='screen',
            parameters=[config_dict],
        ) for config_dict in config_dicts]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config0",
            default_value="sigma0.yaml",
            description="Sigma7 configurations file name"
        ),
        DeclareLaunchArgument(
            "config1",
            default_value="",
            description="Sigma7 configurations file name"
        ),
        OpaqueFunction(function=launch_setup)])
