#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class PlannedPathPublisher(Node):

    def __init__(self):
        super().__init__('planned_path_publisher')
        self.publisher_ = self.create_publisher(Marker, 'planned_path_marker', 1)

    def publish_planned_path(self, path):
        print(path)

        marker = Marker()
        marker.header.frame_id = 'base_link'  # Replace with desired frame ID
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # Thickness of the line

        for point in path.joint_trajectory.points:
            marker.points.append(Point(*point.positions))  # Assuming joint positions represent Cartesian coordinates

        self.publisher_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    planned_path_publisher = PlannedPathPublisher()

    planned_path = RobotTrajectory()

    planned_path_publisher.publish_planned_path(planned_path)

    rclpy.spin(planned_path_publisher)
    planned_path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
