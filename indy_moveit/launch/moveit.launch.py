# import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    prefix = LaunchConfiguration("prefix")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # Planning Configuration
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #         "planner_configs": {
    #             "indy_manipulator": {
    #                 "type": "geometric::RRTConnect",
    #                 "range": 0.2, # Maximum distance between waypoints
    #                 "goal_bias": 0.05,
    #                 "thread_count": 1,
    #                 "simplify": True,
    #                 "simplify_solutions": True,
    #                 "optimization_objectives": ["path_length"],
    #             }
    #         }
    #     }
    # }
    
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
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
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
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
            {"use_sim_time": use_sim_time},
        ],
    )

    # rviz with moveit configuration
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
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
        ],
        output="screen",
    )

    nodes_to_start = [
        move_group_node,
        rviz_node,
        # servo_node
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
            choices=["indy7", "indy7_v2", "indy12", "indyrp2", "indyrp2_v2"]
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
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz_moveit", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
