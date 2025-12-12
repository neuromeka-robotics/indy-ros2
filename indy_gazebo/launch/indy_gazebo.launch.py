import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution #, IfElseSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')
    gazebo_package = FindPackageShare('indy_gazebo')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # gazebo_gui = LaunchConfiguration("gazebo_gui")
    # world_file = LaunchConfiguration("world_file")

    if (indy_type.perform(context) == 'indyrp2') or (indy_type.perform(context) == 'indyrp2_v2'):
        initial_joint_controllers = PathJoinSubstitution(
            [gazebo_package, "controller", "indy_controllers_7dof.yaml"]
        )
    else:
        initial_joint_controllers = PathJoinSubstitution(
            [gazebo_package, "controller", "indy_controllers_6dof.yaml"]
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", "indy.urdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "indy_type:=",
            indy_type,
            " ",
            "indy_eye:=",
            indy_eye,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "indy.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[('gz_args', [' -r -v 4 ', PathJoinSubstitution([FindPackageShare('indy_gazebo'), 'worlds', 'camera_world.sdf'])])]
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content,
                   '-name', 'indy',
                   '-allow_renaming', 'false'],
    )
    
    bridge_params = os.path.join(get_package_share_directory('indy_gazebo'),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay start joint_state_broadcaster
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_controller_spawner],
        )
    )

    # Delay rviz
    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,
        ros_gz_bridge,
        # joint_state_broadcaster_spawner,
        # joint_controller_spawner,
        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_rviz2_spawner,
    ]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2" , "indy12", "indy12_v2", "indyrp2", "indyrp2_v2", "icon7l", "icon3", "nuri3s", "nuri4s", "nuri7c", "nuri20c", "opti5"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye",
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
    