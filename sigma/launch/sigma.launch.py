from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sigma',
            executable='master',
            name='sigma',
            output='screen',
            parameters=[
                {'frequency': 100}, #500
                {'enable_gripper_button': True},
                {'lock_orientation': True},
                {'wrench_topic': '/sigma/force_feedback'}
            ]
        )
    ])
