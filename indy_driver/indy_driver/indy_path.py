#!/usr/bin/env python3

'''
This script subscribes to the TF transform between the base frame and the TCP frame,
and publishes the path of the TCP in the base frame as a nav_msgs/Path message.
'''

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TcpPathPublisher(Node):
    def __init__(self):
        super().__init__('tcp_path_publisher')

        self.base_frame = 'link0'
        self.tcp_frame = 'tcp'

        self.path_pub = self.create_publisher(
            Path,
            '/tcp_path',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.path = Path()
        self.path.header.frame_id = self.base_frame

        # Update the path at a fixed rate 
        self.timer = self.create_timer(
            0.05,
            self.update_path
        )

        # Minimum distance between two points, in meters
        self.min_distance = 0.002

    def update_path(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time()
            )
        except Exception as error:
            self.get_logger().warn(
                f'Failed to lookup TF {self.base_frame} -> '
                f'{self.tcp_frame}: {error}'
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame

        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z

        pose.pose.orientation = transform.transform.rotation

        # Check if the new point is far enough from the last point
        if self.path.poses:
            previous = self.path.poses[-1].pose.position
            current = pose.pose.position

            distance_squared = (
                (current.x - previous.x) ** 2
                + (current.y - previous.y) ** 2
                + (current.z - previous.z) ** 2
            )

            if distance_squared < self.min_distance ** 2:
                return

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)

        # Limit the number of points
        max_points = 10000
        if len(self.path.poses) > max_points:
            self.path.poses = self.path.poses[-max_points:]

        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)

    node = TcpPathPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
