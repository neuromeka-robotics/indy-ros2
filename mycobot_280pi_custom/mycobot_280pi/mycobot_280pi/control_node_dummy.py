#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Control node for Dummy Mycobot
    Please refer to the readme file.
    Author: Nguyen Pham
"""

import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, UInt16, Bool
from sensor_msgs.msg import JointState, Joy


class MyCobot_Comm(Node):
    
    CONTROL_FREQUENCY = 20 # Hz

    HOME_POS_degrees = [0, 8, -127, 40, 0, 0]
    HOME_POS_radians = [0, 0.12, -2.2, 0.7, 0, 0]
    SPEED = 50
    MODE = 0

    GRIPPER_SPEED = 50
    MAX_GRIPPER = 100

    def __init__(self):
        super().__init__("mycobot_comm_task")
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.declare_parameter('robot', "robot0")
        robot_name = self.get_parameter('robot').get_parameter_value().string_value + "_"

        self.control_coords_sub = self.create_subscription(
            Float32MultiArray,
            robot_name + "control_coords",
            self.control_coords_callback,
            qos_profile
        )
        self.control_coords_sub

        self.control_angles_sub = self.create_subscription(
            Float32MultiArray,
            robot_name + "control_angles",
            self.control_angles_callback,
            qos_profile
        )
        self.control_angles_sub

        self.control_radians_sub = self.create_subscription(
            Float32MultiArray,
            robot_name + "control_radians",
            self.control_radians_callback,
            qos_profile
        )
        self.control_radians_sub

        self.control_gripper_sub = self.create_subscription(
            Bool,
            robot_name + "control_gripper",
            self.control_gripper_callback,
            qos_profile
        )
        self.control_gripper_sub

        self.control_gripper_range_sub = self.create_subscription(
            UInt16,
            robot_name + "control_gripper_range",
            self.control_gripper_range_callback,
            qos_profile
        )
        self.control_gripper_range_sub

        self.timer_ = self.create_timer(1/self.CONTROL_FREQUENCY, self.timer_callback)  # Publish every 0.05 second
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            robot_name + "joint2_to_joint1",
            robot_name + "joint3_to_joint2",
            robot_name + "joint4_to_joint3",
            robot_name + "joint5_to_joint4",
            robot_name + "joint6_to_joint5",
            robot_name + "joint6output_to_joint6",
            robot_name + "gripper_controller",
            robot_name + "gripper_right3_to_gripper_right1",
            robot_name + "gripper_base_to_gripper_left2",
            robot_name + "gripper_left3_to_gripper_left1",
            robot_name + "gripper_base_to_gripper_right3",
            robot_name + "gripper_base_to_gripper_right2",
        ]
        self.joint_state_msg.velocity = [0.0,]
        self.joint_state_msg.effort = []

        self.joint_poses = [float(value) for value in self.HOME_POS_radians]
        self.joint_poses.extend([float(value) for value in ([0] * 6)])

        self.res = [0] * 6
        self.record_coords = [self.res, self.SPEED, self.MODE]
        self.angles = None
        self.radians = None
        self.gripper = None
        self.gripper_range = None

        self.coord_flag = False
        self.angle_flag = False
        self.radian_flag = False
        self.gripper_flag = False
        self.gripper_range_flag = False

    def control_coords_callback(self, msg):
        self.record_coords[0] = msg.data
        self.coord_flag = True

    def control_angles_callback(self, msg):
        self.angles  = [msg.data, self.SPEED]
        self.angle_flag = True

    def control_radians_callback(self, msg):
        self.radians  = [msg.data, self.SPEED]
        self.radian_flag = True

    def control_gripper_callback(self, msg):
        self.gripper = msg.data
        self.gripper_flag = True

    def control_gripper_range_callback(self, msg):
        self.gripper_range = msg.data if msg.data <= self.MAX_GRIPPER else self.MAX_GRIPPER
        self.gripper_range_flag = True

    def send_cmds(self):
        if self.coord_flag:
            self.coord_flag = False
        elif self.angle_flag:
            self.angle_flag = False
        elif self.radian_flag:
            for i in range (len(self.radians[0])):
                self.joint_poses[i] = float(self.radians[0][i])
                #self.joint_poses = [float(value) for value in self.radians[0]]
            self.radian_flag = False
        elif self.gripper_flag:
            self.gripper_flag = False
        elif self.gripper_range_flag:
            # gripper_value = (self.gripper_range / 100) * -0.85 + 0.15
            gripper_value = (self.gripper_range / 100) * 0.85 - 0.7
            for i in range (6, 12):
                self.joint_poses[i] = gripper_value if i < 9 else -gripper_value
            self.gripper_range_flag = False

    def joint_state_publisher(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.joint_poses
        self.joint_state_pub.publish(self.joint_state_msg)

    # Timer callback for publish
    def timer_callback(self):
        self.joint_state_publisher()
        self.send_cmds()


def main(args=None):
    rclpy.init(args=args)
    mycobot = MyCobot_Comm()
    
    rclpy.spin(mycobot)
    
    mycobot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
