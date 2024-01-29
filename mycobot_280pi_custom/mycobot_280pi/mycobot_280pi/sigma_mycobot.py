#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Sigma and Mycobot Interface
    Please refer to the readme file.
    Author: Nguyen Pham
"""

import os 
import rclpy
import math
import yaml

from rclpy.node import Node
# from rclpy.time import Time, Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Float32MultiArray, Bool, UInt16
from geometry_msgs.msg import TwistStamped, PoseStamped


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians


class Sigma_MyCobot_Comm(Node):

    # MIN_LEN = 15
    # MAX_LEN = 220 # mm
    # MAX_ANGLE = 180 # deg
    # LEN_TO_UPDATE = 3 # mm
    # ANGLE_TO_UPDATE = 3 # deg

    def __init__(self):
        super().__init__("sigma_mycobot_comm")
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.declare_parameter('robot', "robot0")
        self.declare_parameter('sigma', "sigma0")
        robot = self.get_parameter('robot').get_parameter_value().string_value + "_"
        sigma = self.get_parameter('sigma').get_parameter_value().string_value

        self.sigma_init_pose = None

        self.sigma_sub = self.create_subscription(
            PoseStamped,
            sigma + "/pose",
            self.sigma_callback,
            qos_profile
        )
        self.sigma_sub
        
        self.sigma_gripper_sub = self.create_subscription(
            Float32,
            sigma + "/gripper_angle",
            self.sigma_gripper_callback,
            qos_profile
        )
        self.sigma_gripper_sub

        self.gripper = 0.0
        self.prev_gripper = 0.0

        # Limit to 20Hz
        self.timer_ = self.create_timer(0.05, self.timer_callback)  # Publish every 0.05 second
        self.coords_pub = self.create_publisher(Float32MultiArray, robot + 'control_coords', qos_profile)
        self.angles_pub = self.create_publisher(Float32MultiArray, robot + 'control_angles', qos_profile)
        # self.gripper_pub = self.create_publisher(Bool, robot + '/control_gripper', qos_profile)
        self.gripper_pub = self.create_publisher(UInt16, robot + 'control_gripper_range', qos_profile)

        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        file_path = dir_path + "/config/workspace_params.yaml"
        with open(file_path, 'r') as file:
            self.config_yaml_info = yaml.safe_load(file)

        self.get_logger().info(f"Home coords: {self.config_yaml_info['home_coords']}")
        self.get_logger().info(f"Sigma Workspace: {self.config_yaml_info['sigma_workspace']}")
        self.get_logger().info(f"Robot Workspace: {self.config_yaml_info['robot_workspace']}")
        self.get_logger().info(f"Gripper Angle: {self.config_yaml_info['gripper_angle']}")
        self.get_logger().info(f"Robot Config: {self.config_yaml_info['robot_config']}")

        self.coords = self.config_yaml_info['home_coords'].copy() # homepos
        self.prev_coords = self.coords.copy()
        

    def sigma_callback(self, msg):
        # Get the init pose
        if self.sigma_init_pose is None:
            roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            self.sigma_init_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw]
        self.coords = self.config_yaml_info['home_coords'].copy()

        # Sigma offset
        roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        offset_x = msg.pose.position.x - self.sigma_init_pose[0]
        offset_y = msg.pose.position.y - self.sigma_init_pose[1]
        offset_z = msg.pose.position.z - self.sigma_init_pose[2]
        offset_roll = roll - self.sigma_init_pose[3]
        offset_pitch = pitch - self.sigma_init_pose[4]
        offset_yaw = yaw - self.sigma_init_pose[5]
        # self.get_logger().info(f"x: {offset_x} y: {offset_y} z: {offset_z} u: {offset_roll} v: {offset_pitch} w: {offset_yaw}")
        # self.get_logger().info(f"coords: {self.coords}")
        # self.get_logger().info(f"sigma: {self.sigma_init_pose}")
        #self.get_logger().info(f"sigma: {roll}, {pitch}, {yaw}")
        
        # Add to coords
        self.coords[0] -= ((offset_y * 1000) / 
                       self.config_yaml_info['sigma_workspace']['translation'][1] * 
                       self.config_yaml_info['robot_workspace']['translation'][1])
        self.coords[1] += ((offset_x * 1000) / 
                       self.config_yaml_info['sigma_workspace']['translation'][0] * 
                       self.config_yaml_info['robot_workspace']['translation'][0])
        self.coords[2] += ((offset_z * 1000) / 
                       self.config_yaml_info['sigma_workspace']['translation'][2] * 
                       self.config_yaml_info['robot_workspace']['translation'][2])
        self.coords[4] -= ((offset_roll / math.pi * 180) /
                        self.config_yaml_info['sigma_workspace']['rotation'][0] * 
                        self.config_yaml_info['robot_workspace']['rotation'][0])
        self.coords[3] += ((offset_pitch / math.pi * 180) /
                        self.config_yaml_info['sigma_workspace']['rotation'][1] * 
                        self.config_yaml_info['robot_workspace']['rotation'][1])
        self.coords[5] += ((offset_yaw / math.pi * 80) /
                        self.config_yaml_info['sigma_workspace']['rotation'][2] * 
                        self.config_yaml_info['robot_workspace']['rotation'][2])

        # Clamp the position within the limits
        self.coords[0] = min(max(self.coords[0], -self.config_yaml_info['robot_config']['max_len']), self.config_yaml_info['robot_config']['max_len'])
        self.coords[1] = min(max(self.coords[1], -self.config_yaml_info['robot_config']['max_len']), self.config_yaml_info['robot_config']['max_len'])
        self.coords[2] = min(max(self.coords[2], self.config_yaml_info['robot_config']['min_len']), self.config_yaml_info['robot_config']['max_len'])
        self.coords[3] = min(max(self.coords[3], -self.config_yaml_info['robot_config']['max_angle']), self.config_yaml_info['robot_config']['max_angle'])
        self.coords[4] = min(max(self.coords[4], -self.config_yaml_info['robot_config']['max_angle']), self.config_yaml_info['robot_config']['max_angle'])
        self.coords[5] = min(max(self.coords[5], -self.config_yaml_info['robot_config']['max_angle']), self.config_yaml_info['robot_config']['max_angle'])

        # self.get_logger().info(f"new coord: {self.coords}")

    def sigma_gripper_callback(self, msg):
        self.gripper = (msg.data / 
                    self.config_yaml_info['gripper_angle']['sigma'] *
                    self.config_yaml_info['gripper_angle']['robot'])

    def timer_callback(self):

        # Check if coordinates data has changed
        if abs(self.coords[0] - self.prev_coords[0]) >= self.config_yaml_info['robot_config']['len_to_update'] \
            or abs(self.coords[1] - self.prev_coords[1]) >= self.config_yaml_info['robot_config']['len_to_update'] \
            or abs(self.coords[2] - self.prev_coords[2]) >= self.config_yaml_info['robot_config']['len_to_update'] \
            or abs(self.coords[3] - self.prev_coords[3]) >= self.config_yaml_info['robot_config']['angle_to_update'] \
            or abs(self.coords[4] - self.prev_coords[4]) >= self.config_yaml_info['robot_config']['angle_to_update'] \
            or abs(self.coords[5] - self.prev_coords[5]) >= self.config_yaml_info['robot_config']['angle_to_update']:
            msg = Float32MultiArray()
            float_coords = [float(value) for value in self.coords]
            msg.data = float_coords
            self.coords_pub.publish(msg)
            self.prev_coords = self.coords.copy()

        if self.gripper != self.prev_gripper:
            msg = UInt16()
            msg.data = int(self.gripper)
            self.gripper_pub.publish(msg)
            self.prev_gripper = self.gripper

        # Check if gripper value has changed
        # if self.gripper >= 0.3 and self.prev_gripper < 0.3:
        #     gripper_value = False
        # elif self.gripper < 0.3 and self.prev_gripper >= 0.3:
        #     gripper_value = True

        # if gripper_value is not None:
        #     msg = Bool()
        #     msg.data = gripper_value
        #     self.gripper_pub.publish(msg)
        #     self.prev_gripper = self.gripper


def main(args=None):
    rclpy.init(args=args)
    sigma_mycobot = Sigma_MyCobot_Comm()
    
    rclpy.spin(sigma_mycobot)
    
    sigma_mycobot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
