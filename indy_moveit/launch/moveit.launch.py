# import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from indy_moveit.launch_common import load_yaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')
    moveit_config_package = FindPackageShare('indy_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    servo_mode = LaunchConfiguration("servo_mode")
    prefix = LaunchConfiguration("prefix")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_3d_perception = LaunchConfiguration("enable_3d_perception")

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
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_config_package, "srdf", "indy.srdf.xacro"]),
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
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [moveit_config_package, "moveit_config", "kinematics.yaml"]
    )

    if (indy_type.perform(context) == 'indyrp2') or (indy_type.perform(context) == 'indyrp2_v2'):
        joint_limit_yaml = load_yaml("indy_moveit", "moveit_config/joint_limits_7dof.yaml")
    else:
        joint_limit_yaml = load_yaml("indy_moveit", "moveit_config/joint_limits_6dof.yaml")
    robot_description_planning = {
        "robot_description_planning": joint_limit_yaml
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    ompl_planning_yaml = load_yaml("indy_moveit", "moveit_config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    if (indy_type.perform(context) == 'indyrp2') or (indy_type.perform(context) == 'indyrp2_v2'):
        controllers_yaml = load_yaml("indy_moveit", "moveit_config/controllers_7dof.yaml")
    else:
        controllers_yaml = load_yaml("indy_moveit", "moveit_config/controllers_6dof.yaml")

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
    
    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers,
        planning_scene_monitor_parameters,
        {"use_sim_time": use_sim_time},
    ]

    if enable_3d_perception.perform(context).lower() in ("true", "1"):
        sensors_3d_yaml = load_yaml("indy_moveit", "moveit_config/sensor_3d.yaml") or {}
        sensor_entries = {
            key: value for key, value in sensors_3d_yaml.items() if key != "octomap"
        }
        if sensor_entries:
            move_group_parameters.append(sensor_entries)

        octomap_cfg = sensors_3d_yaml.get("octomap", {})
        if octomap_cfg:
            move_group_parameters.append(
                {
                    "octomap_frame": octomap_cfg.get("frame", "world"),
                    "octomap_resolution": octomap_cfg.get("resolution", 0.05),
                    "max_range": octomap_cfg.get("max_range", 5.0),
                }
            )
    
    # Start the actual move_group node/action server  
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_parameters,
    )

    # rviz with moveit configuration
    if servo_mode.perform(context) == 'true':
        rviz_config_file = PathJoinSubstitution(
            [moveit_config_package, "rviz_config", "indy_servo.rviz"] #indy_servo
        )    
    else:
        rviz_config_file = PathJoinSubstitution(
            [moveit_config_package, "rviz_config", "indy_moveit.rviz"]
        )

    rviz_node = Node(
        condition=IfCondition(launch_rviz_moveit),
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

    # Servo node for realtime control
    servo_yaml = load_yaml("indy_moveit", "moveit_config/indy_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
        output="screen",
    )

    container = ComposableNodeContainer(
        name="moveit_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # ComposableNode(
            #     package="indy_moveit",
            #     plugin="moveit_servo::JoyToServoPub",
            #     name="controller_to_servo_node",
            # ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
                # extra_arguments=[{
                #     "use_intra_process_comms": True,
                #     "deadzone": 0.35
                # }],
            )
        ],
        output="screen",
    )

    if servo_mode.perform(context) == 'true':        
        nodes_to_start = [servo_node, container, rviz_node]
    else:
        nodes_to_start = [move_group_node, rviz_node]
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
            "servo_mode",
            default_value="false",
            description="Servoing mode",
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
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz_moveit", default_value="true", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_3d_perception",
            default_value="false",
            description="Enable MoveIt Octomap updates from depth sensors.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
