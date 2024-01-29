#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Control node for Mycobot 280pi
    Please refer to the readme file.
    Author: Nguyen Pham from Neuromeka
"""

import rclpy
import math
import time
import random
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, UInt16, Bool
from sensor_msgs.msg import JointState, Joy


class MyCobot_Comm(Node):
    
    CONTROL_FREQUENCY = 20 # Hz

    HOME_POS_degrees = [0, 8, -127, 40, 0, 0]
    HOME_POS_radians = [0, 0.12, -2.2, 0.7, 0, 0]
    SPEED = 80
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

        self.joint_poses = [float(0.0)] * len(self.joint_state_msg.name)
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.joint_poses
        self.joint_state_pub.publish(self.joint_state_msg)

        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        home_pose = [self.HOME_POS_radians, self.SPEED]

        print("Move the robot to Home position ...")

        self.mc.send_radians(*home_pose)
        time.sleep(5)

        #while self.mc.is_moving():
        #    time.sleep(0.2)
        
        while True:
            self.res = self.mc.get_coords()
            if self.res:
                break
            time.sleep(0.1)

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

        print("Current coords: %s" % self.record_coords)
        # print("tool reference: ", self.mc.get_tool_reference())
        # print("world reference: ", self.mc.get_world_reference())
        # print("reference frame: ", self.mc.get_reference_frame())
        # print("end type: ", self.mc.get_end_type())

    def control_coords_callback(self, msg):
        # print("coords ", msg.data)
        self.record_coords[0] = msg.data
        self.coord_flag = True

    def control_angles_callback(self, msg):
        # print("angles", msg.data)
        self.angles  = [msg.data, self.SPEED]
        self.angle_flag = True

    def control_radians_callback(self, msg):
        # print("radians ", msg.data)
        self.radians  = [msg.data, self.SPEED]
        self.radian_flag = True

    def control_gripper_callback(self, msg):
        # print("gripper ", msg.data)
        self.gripper = msg.data
        self.gripper_flag = True

    def control_gripper_range_callback(self, msg):
        # print("gripper range ", msg.data)
        self.gripper_range = msg.data if msg.data <= self.MAX_GRIPPER else self.MAX_GRIPPER
        self.gripper_range_flag = True

    def send_cmds(self):
        if self.coord_flag:
            self.mc.send_coords(*self.record_coords)
            self.coord_flag = False
        elif self.angle_flag:
            self.mc.send_angles(*self.angles)
            self.angle_flag = False
        elif self.radian_flag:
            self.mc.send_radians(*self.radians)
            self.radian_flag = False
        elif self.gripper_flag:
            self.mc.set_gripper_state(self.gripper, self.GRIPPER_SPEED)
            self.gripper_flag = False
        elif self.gripper_range_flag:
            self.mc.set_gripper_value(self.gripper_range, self.GRIPPER_SPEED)
            # gripper_value = (self.gripper_range / 100) * -0.85 + 0.15
            gripper_value = (self.gripper_range / 100) * 0.85 - 0.7
            for i in range (6, 12):
                self.joint_poses[i] = gripper_value if i < 9 else -gripper_value
            self.gripper_range_flag = False

    def joint_state_publisher(self):
        radians = self.mc.get_radians()
        data_list = []
        for _, value in enumerate(radians):
            data_list.append(value)

        for i in range(len(data_list)):
            self.joint_poses[i] = data_list[i]

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
