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

    robot_quantity = LaunchConfiguration("robot_quantity")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")

    desc_file_name = "one_indy.urdf.xacro"
    semantic_file_name = "one_indy.srdf.xacro"
    kinematics_file_name = "one_kinematics.yaml"
    controller_file_name = "one_controllers.yaml"
    ompl_file_name = "one_ompl_planning.yaml"

    if int(robot_quantity.perform(context)) == 2:
        desc_file_name = "two_indy.urdf.xacro"
        semantic_file_name = "two_indy.srdf.xacro"
        kinematics_file_name = "two_kinematics.yaml"
        controller_file_name = "two_controllers.yaml"
        ompl_file_name = "two_ompl_planning.yaml"
    elif int(robot_quantity.perform(context)) == 4:
        desc_file_name = "four_indy.urdf.xacro"
        semantic_file_name = "four_indy.srdf.xacro"
        kinematics_file_name = "four_kinematics.yaml"
        controller_file_name = "four_controllers.yaml"
        ompl_file_name = "four_ompl_planning.yaml"
    
    robot_config = {}
    robot_description_parts = []
    robot_semantic_description_parts = []

    for i in range(int(robot_quantity.perform(context))):
        robot_name = LaunchConfiguration(f"robot{i}_name")
        robot_xyz = LaunchConfiguration(f"robot{i}_xyz")
        robot_rpy = LaunchConfiguration(f"robot{i}_rpy")
        indy_type = LaunchConfiguration(f"indy{i}_type")
        indy_eye = LaunchConfiguration(f"indy{i}_eye")
        
        robot_description_parts.extend(
            [
                f" robot{i}_name:={robot_name.perform(context)}_",
                f" origin{i}_xyz:={robot_xyz.perform(context)}",
                f" origin{i}_rpy:={robot_rpy.perform(context)}",
                f" indy{i}_type:={indy_type.perform(context)}",
                f" indy{i}_eye:={indy_eye.perform(context)}",
            ]
        )

        robot_semantic_description_parts.extend(
            [
                f" robot{i}_name:={robot_name.perform(context)}_",
                f" indy{i}_type:={indy_type.perform(context)}",
            ]
        )

        robot_config[f"robot{i}"] = {
            "robot_name": robot_name,
            "indy_type": indy_type,
        }

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", desc_file_name]),
            " ",
        ] + robot_description_parts
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([moveit_config_package, "srdf", semantic_file_name]),
            " ",
        ] + robot_semantic_description_parts
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [moveit_config_package, "moveit_config", kinematics_file_name]
    )

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.5,
        }
    }
    ompl_planning_yaml = load_yaml("indy_moveit", "moveit_config/"+ompl_file_name)
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("indy_moveit", "moveit_config/"+controller_file_name)
    for robot_key, robot_conf in robot_config.items():
        if (robot_conf['indy_type'].perform(context) == 'indyrp2') or (robot_conf['indy_type'].perform(context) == 'indyrp2_v2'):
            controller_name = robot_conf['robot_name'].perform(context) + "_joint_trajectory_controller"
            if controller_name in controllers_yaml["controller_names"]:
                controller = controllers_yaml[controller_name]
                joints = controller.get("joints", [])
                joints.append(robot_conf['robot_name'].perform(context) + "_joint6")
                controller["joints"] = joints
    # print(controllers_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
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

    nodes_to_start = [move_group_node, rviz_node]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    robots = [
        {"name": "robot0", "xyz": '"0 0 0"', "rpy": '"0 0 0"', "indy_type": "indy7", "indy_eye": "false"},
        {"name": "robot1", "xyz": '"0 1 0"', "rpy": '"0 0 0"', "indy_type": "indy12_v2", "indy_eye": "false"},
        {"name": "robot2", "xyz": '"1 1 0"', "rpy": '"0 0 0"', "indy_type": "indy7_v2", "indy_eye": "false"},
        {"name": "robot3", "xyz": '"1 0 0"', "rpy": '"0 0 0"', "indy_type": "indyrp2", "indy_eye": "true"}
    ]

    for i, robot in enumerate(robots):
        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_name",
                default_value=robot['name']
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_xyz",
                default_value=robot['xyz'],
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"{robot['name']}_rpy",
                default_value=robot['rpy'],
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"indy{i}_type",
                default_value=robot['indy_type'],
                description="Type of Indy robot.",
                choices=["indy7", "indy7_v2", "indy12", "indy12_v2", "indyrp2", "indyrp2_v2"]
            )
        )

        declared_arguments.append(
            DeclareLaunchArgument(
                f"indy{i}_eye",
                default_value=robot['indy_eye'],
                description="Work with Indy Eye",
            )
        )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_quantity",
            default_value="1",
            description="Robot quantity. Valid number are 1, 2 and 4",
            choices=["1", "2", "4"]
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
        "launch_rviz_moveit", 
        default_value="true", 
        description="Launch RViz?"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
