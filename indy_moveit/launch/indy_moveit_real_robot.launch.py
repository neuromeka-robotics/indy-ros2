from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    # description_package = FindPackageShare('indy_description')
    indy_driver_package = FindPackageShare('indy_driver')
    moveit_config_package = FindPackageShare('indy_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_ip = LaunchConfiguration("indy_ip")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    servo_mode = LaunchConfiguration("servo_mode")
    prefix = LaunchConfiguration("prefix")
    enable_realsense = LaunchConfiguration("enable_realsense")
    realsense_namespace = LaunchConfiguration("realsense_namespace")
    camera_parent_frame = LaunchConfiguration("camera_parent_frame")
    camera_link_frame = LaunchConfiguration("camera_link_frame")
    camera_pose_x = LaunchConfiguration("camera_pose_x")
    camera_pose_y = LaunchConfiguration("camera_pose_y")
    camera_pose_z = LaunchConfiguration("camera_pose_z")
    camera_pose_qx = LaunchConfiguration("camera_pose_qx")
    camera_pose_qy = LaunchConfiguration("camera_pose_qy")
    camera_pose_qz = LaunchConfiguration("camera_pose_qz")
    camera_pose_qw = LaunchConfiguration("camera_pose_qw")

    indy_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [indy_driver_package, "/launch", "/indy_bringup.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_ip": indy_ip,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "prefix": prefix,
            "launch_rviz": "false",
        }.items(),
    )

    indy_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "servo_mode": servo_mode,
            "prefix": prefix,
            "use_sim_time": "false",
            "launch_rviz_moveit": "true", # if name == "launch_rviz" => spawn 2 rviz
            "enable_3d_perception": enable_realsense,
        }.items(),
    )

    nodes_to_launch = [
        indy_bringup_launch,
        indy_moveit_launch,
    ]

    if enable_realsense.perform(context).lower() in ("true", "1"):
        realsense_package = FindPackageShare('realsense2_camera')
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [realsense_package, "/launch", "/rs_launch.py"]
            ),
            launch_arguments={
                "camera_name": realsense_namespace,
                "pointcloud.enable": "true",
                "align_depth.enable": "true",
                # "rgb_camera.profile": "640x480x30",
                # "depth_module.profile": "640x480x15",
            }.items(),
        )
        nodes_to_launch.append(realsense_launch)

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="indy_realsense_static_tf",
            output="log",
            arguments=[
                camera_pose_x,
                camera_pose_y,
                camera_pose_z,
                camera_pose_qx,
                camera_pose_qy,
                camera_pose_qz,
                camera_pose_qw,
                camera_parent_frame,
                camera_link_frame,
            ],
        )
        nodes_to_launch.append(static_tf)

    return nodes_to_launch


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
            "enable_realsense",
            default_value="false",
            description="Launch an Intel RealSense depth camera pipeline alongside the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_namespace",
            default_value="camera",
            description="Namespace/camera_name used by the RealSense driver.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_parent_frame",
            default_value="world",
            description="Fixed frame the camera is mounted to.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_link_frame",
            default_value="camera_link",
            description="Frame broadcast by the RealSense driver for the camera link.",
        )
    )

    declared_arguments.extend(
        [
            DeclareLaunchArgument("camera_pose_x", default_value="0.0", description="Camera X offset (m)."),
            DeclareLaunchArgument("camera_pose_y", default_value="0.0", description="Camera Y offset (m)."),
            DeclareLaunchArgument("camera_pose_z", default_value="1.0", description="Camera Z offset (m)."),
            DeclareLaunchArgument("camera_pose_qx", default_value="0.0", description="Camera quaternion qx."),
            DeclareLaunchArgument("camera_pose_qy", default_value="0.0", description="Camera quaternion qy."),
            DeclareLaunchArgument("camera_pose_qz", default_value="0.0", description="Camera quaternion qz."),
            DeclareLaunchArgument("camera_pose_qw", default_value="1.0", description="Camera quaternion qw."),
        ]
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])