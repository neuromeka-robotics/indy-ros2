from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare('indy_description')
    indy_driver_package = FindPackageShare('indy_driver')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_ip = LaunchConfiguration("indy_ip")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    if (indy_type.perform(context) == 'indyrp2') or (indy_type.perform(context) == 'indyrp2_v2'):
        initial_joint_controllers = PathJoinSubstitution(
            [indy_driver_package, "controller", "indy_controllers_7dof.yaml"]
        )
    else:
        initial_joint_controllers = PathJoinSubstitution(
            [indy_driver_package, "controller", "indy_controllers_6dof.yaml"]
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
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "indy.rviz"]
    )

    indy_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    indy_driver = Node(
        package="indy_driver",
        executable="indy_driver.py",
        name="indy_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            {'indy_ip': indy_ip.perform(context)},
            {'indy_type': indy_type.perform(context)},
        ],
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay rviz
    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher_node,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        indy_driver,
        # indy_control_node,
        robot_state_publisher_node,
        # delay_rviz2_spawner
        rviz_node
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
            "indy_ip", 
            description="IP address for real robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2" , "indy12", "indy12_v2", "indyrp2", "indyrp2_v2"]
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
