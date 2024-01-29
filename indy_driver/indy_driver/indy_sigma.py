#!/usr/bin/python3
#-*- coding: utf-8 -*-
'''
    Neuromeka V-SCRC
    Nguyen Pham
'''

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Float32MultiArray, Bool, Float32
from geometry_msgs.msg import TwistStamped, PoseStamped
from ament_index_python.packages import get_package_share_directory

import sys
import os
import time
__nrmk_path = os.path.join(get_package_share_directory('indy_driver'), "nrmk_utils")
sys.path.append(__nrmk_path)
from math_tools import *
from file_io import *
from datetime import datetime
from datetime import timedelta


TMP_LOG_PATH = os.path.join(get_home_path(), "TMP_LOG")
if not os.path.isdir(TMP_LOG_PATH):
    os.mkdir(TMP_LOG_PATH)


class IndySigmaConnection(Node):
    def __init__(self):
        super().__init__('indy_sigma')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        self.declare_parameter('sigma_name', "sigma0")
        self.declare_parameter('robot0', "robot0")
        self.declare_parameter('robot1', "robot1")
        self.declare_parameter('origin.xyz', [0.0,0.0,0.0])
        self.declare_parameter('origin.rpy', [0.0,0.0,0.0])
        self.declare_parameter('pub_rate', 20.0)
        self.declare_parameter('finger_thresh', 0.2)
        self.sigma_name = self.get_parameter('sigma_name').get_parameter_value().string_value
        self.robot0_name = self.get_parameter('robot0').get_parameter_value().string_value
        self.robot1_name = self.get_parameter('robot1').get_parameter_value().string_value
        self.origin_xyz = self.get_parameter('origin.xyz').get_parameter_value().double_array_value
        self.origin_rpy = self.get_parameter('origin.rpy').get_parameter_value().double_array_value
        self.pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value
        self.finger_thresh = self.get_parameter('finger_thresh').get_parameter_value().double_value

        self.get_logger().info(f"MASTER NAME: {self.sigma_name}")
        self.get_logger().info(f"SLAVES: {self.robot0_name}, {self.robot1_name}")
        self.get_logger().info(f"ORIGIN: {self.origin_xyz} / {self.origin_rpy}")
        self.get_logger().info(f"PUBLISH RATE: {self.pub_rate} Hz")
        

        self.Rgm = Rot_rpy(*self.origin_rpy)        # global-robot R Roll, Pitch, Ywa (RPY) 회전 각도를 이용, 로봇의 전역 좌표계에서 로봇의 회전을 나타냄
        self.Rmg = self.Rgm.transpose()             # robot-global R 전치 행렬을 통해 로봇의 회전을 전역 좌표계에서 로봇 좌표계로 변환 R : Rotation(회전)- 물체나 좌표 공간의 방향을 변환
        self.Tgm = SE3(self.Rgm, self.origin_xyz)   # global-robot T 로봇의 원래 위치와 자세를 나타내는 변환 행렬, 원점 좌표를 사용하여 전역 좌표계에서 로봇의 위치와 자세를 나타냄
        self.Tmg = np.linalg.inv(self.Tgm)          # robot-global T 로봇-전역 T의 역행렬을 구하여, 로봇 좌표계에서의 위치와 자세를 전역 좌표계로 변환합니다.
        self.pos_scale, self.rot_scale = 1.0, 0.3
        self.Rmv_ref = np.identity(3)               # reference virtual is always aligned with master
        
        # Initialize variable
        self.save_log = False
        self.Tmh_ref = None
        self.tele_mode = False
        self.enable_tele = False
        self.is_tele_enabled = False
        self.current_robot = self.robot0_name
        self.other_robot = self.robot1_name
        self.start_time = time.time_ns()
        self.elapsed_time = 0

        self.target_offset = [0]*6
        self.target_offset_last = [0] * 6
        self.Pmh = [0]*3
        self.Rmh =  np.zeros((3,3))



        self.sigma_log = []
        self.tele_move_log = []
        self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
        self.sigma_labels = ["Time", "X", "Y", "Z", "U","V","W"]  # Add labels for tele_move_log columns
        self.tele_move_labels = ["Time", "X", "Y", "Z", "R", "P", "Y"]  # Add labels for sigma_log columns

        self.save_log_sub = self.create_subscription(
            Bool,
            'save_log_status',
            self.save_log_callback,
            qos_profile)

        self.sigma_sub = self.create_subscription(
            PoseStamped,
            self.sigma_name + "/pose",
            self.sigma_callback,
            qos_profile
        )
        
        self.sigma_gripper_sub = self.create_subscription(
            Float32,
            self.sigma_name + "/gripper_angle",
            self.sigma_gripper_callback,
            qos_profile
        )

        self.sigma_switch_robot_sub = self.create_subscription(
            Bool,
            self.sigma_name + "_switch_robot",
            self.sigma_switch_robot_callback,
            qos_profile
        )

        self.sigma_tele_mode_sub = self.create_subscription(
            Bool,
            self.sigma_name + "_tele_mode",
            self.sigma_telemode_callback,
            qos_profile
        )

        self.sigma_scale_sub = self.create_subscription(
            Float32MultiArray,
            self.sigma_name + "_scale",
            self.sigma_scale_callback,
            qos_profile
        )

        self.sigma_button_sub = self.create_subscription(
            Bool,
            self.sigma_name + "/button",
            self.sigma_button_callback,
            qos_profile
        )

        # Initialize topics
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)

        self.tele0_mode_pub = self.create_publisher(Bool, self.robot0_name + '_tele_mode', qos_profile)
        self.tele0_move_pub = self.create_publisher(Float32MultiArray, self.robot0_name + '_tele_move', qos_profile)
        self.tele0_finger_pub = self.create_publisher(Bool, self.robot0_name + '_tele_finger', qos_profile)

        self.tele1_mode_pub = self.create_publisher(Bool, self.robot1_name + '_tele_mode', qos_profile)
        self.tele1_move_pub = self.create_publisher(Float32MultiArray, self.robot1_name + '_tele_move', qos_profile)
        self.tele1_finger_pub = self.create_publisher(Bool, self.robot1_name + '_tele_finger', qos_profile)

        self.tele_mode_msg = Bool()
        self.tele_move_msg = Float32MultiArray()

        self.robot_pubs = {
            self.robot0_name: {
                "tele_mode_pub": self.tele0_mode_pub,
                "tele_move_pub": self.tele0_move_pub,
                "finger_pub": self.tele0_finger_pub
            },
            self.robot1_name: {
                "tele_mode_pub": self.tele1_mode_pub,
                "tele_move_pub": self.tele1_move_pub,
                "finger_pub": self.tele1_finger_pub
            }
        }

    def sigma_gripper_callback(self, msg):
         finger = abs(float(msg.data))
         # self.enable_tele = finger < self.finger_thresh

    def sigma_button_callback(self, msg):
         if msg.data is True : 
            self.enable_tele = True
         elif msg.data is False :
            self.enable_tele = False

    def save_log_callback(self, msg):
        if msg.data is True:
            self.save_log = True
        elif msg.data is False:
            self.save_log = False

    '''
    Sigma subscribe
    '''
    def sigma_switch_robot_callback(self, msg):
        restart_teleop = self.tele_mode
        if self.tele_mode:
            self.pub_stop_teleop()
        self.current_robot, self.other_robot = self.other_robot, self.current_robot
        if restart_teleop:
            self.pub_start_teleop()
        self.get_logger().warning(f"SWITCH TO {self.current_robot.upper()}")

    def sigma_telemode_callback(self, msg):
        if msg.data is True and self.tele_mode is False:
            self.pub_start_teleop()
        elif msg.data is False and self.tele_mode is True:
            self.pub_stop_teleop()
            
    def pub_start_teleop(self):
        self.tele_mode_msg.data = self.tele_mode = True
        self.Tmh_ref = None
        self.target_offset = [0]*6
        self.target_offset_last = [0] * 6
        self.robot_pubs[self.current_robot]["tele_mode_pub"].publish(self.tele_mode_msg)
        self.get_logger().warning(f"Turn ON Tele Mode FOR {self.current_robot.upper()}")
        
    def pub_stop_teleop(self):
        self.tele_mode_msg.data = self.tele_mode = False
        self.Tmh_ref = None
        self.target_offset = [0]*6
        self.target_offset_last = [0] * 6
        self.robot_pubs[self.current_robot]["tele_mode_pub"].publish(self.tele_mode_msg)
        self.get_logger().warning(f"Turn OFF Tele Mode FOR {self.current_robot.upper()}")

    def sigma_scale_callback(self, msg):
        self.pos_scale, self.rot_scale = msg.data

    def sigma_callback(self, msg):
        if self.tele_mode:   # 만약 텔레오퍼레이션 모드가 활성화되어 있다면:
            # master-to-handle input, sigma pose정보를 가져옴
            self.Pmh = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            self.Rmh = Rot_quat(msg.pose.orientation.x, msg.pose.orientation.y,
                           msg.pose.orientation.z, msg.pose.orientation.w)
            # self.get_logger().info(f"Pmh value is {Pmh}")
            # self.get_logger().info(f"Rmh value is {Rmh}")
            self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
            
            # Get the init pose
            if self.Tmh_ref is None:
                # master-to-handle reference
                self.Pmh_ref = self.Pmh # 초기 pose 정보
                self.Rmh_ref = self.Rmh # 초기 rotation 정보
                self.Tmh_ref = SE3(self.Rmh_ref, self.Pmh_ref) # 초기 위치와 회전을 나타내는 변환 행렬 생성
                self.Rhv = np.matmul(self.Rmh_ref.transpose(), self.Rmv_ref)    # handle-to-virtual 
            else:
                if self.is_tele_enabled:
                    # Calc new Position vector in global 새로운 위치와 회전 정보를 계산하고 로봇을 이동시킴
                    pvec_m = np.subtract(self.Pmh, self.Pmh_ref)  # 마스터 기준 변위
                    pvec_g = np.matmul(self.Rgm, pvec_m)  # 글로벌 좌표계 기준 변위

                    # Calc new Rotation vector in global
                    Rmv = np.matmul(self.Rmh, self.Rhv)                  # master-to-new-virtual rotation
                    dRv = np.matmul(self.Rmv_ref.transpose(), Rmv)  # virtual rotation ref-to-new
                    rvec_v = Rotation.from_matrix(dRv).as_rotvec()  # rotation vector in virtual
                    rvec_m = np.matmul(self.Rmv_ref, rvec_v)        # rotation vector in master
                    rvec_g = np.matmul(self.Rgm, rvec_m)            # rotation vector in global
                    dRv_g = Rotation.from_rotvec(rvec_g).as_matrix()  # Rotation matrix in global

                    # 이전 회전 정보와 현재 회전 정보를 결합
                    rvec_g_last = np.multiply(np.reciprocal(self.rot_scale),self.target_offset_last[3:])
                    dRv_g_last = Rotation.from_rotvec(rvec_g_last).as_matrix()
                    dRv_g_acum = np.matmul(dRv_g, dRv_g_last)
                    rvec_g_acum = Rotation.from_matrix(dRv_g_acum).as_rotvec()
                    # 로봇의 목표 위치 및 회전 정보 업데이트
                    # self.get_logger().info(f"rvec_g: {np.round(rvec_g, 3)}")
                    # self.get_logger().info(f"rvec_l: {np.round(rvec_g_last, 3)}")
                    # self.get_logger().info(f"rvec_a: {np.round(rvec_g_acum, 3)}")
                    self.target_offset[:3] = np.add(self.target_offset_last[:3], np.multiply(self.pos_scale, pvec_g))
                    # 목표 위치 정보를 나타내며, pvec_g는 새로 계산된 위치 정보입니다. 이 위치 정보는 이전 위치 정보(self.target_offset_last[:3])와 새로 계산된 위치 정보(np.multiply(self.pos_scale, pvec_g))를 합하여 업데이트
                    self.target_offset[3:] = np.multiply(self.rot_scale, rvec_g_acum)
                    # 목표 회전 정보를 나타내며, rvec_g_acum는 새로 계산된 회전 정보입니다. 이 회전 정보는 np.multiply(self.rot_scale, rvec_g_acum)를 통해 업데이트
                    # self.get_logger().info(f"target_offset: {np.round(self.target_offset, 3)}")
                    # 텔레오퍼레이션 모드가 비활성화된 경우 로봇을 정지
                    if not self.enable_tele:
                        self.is_tele_enabled = self.enable_tele
                        self.robot_pubs[self.current_robot]["finger_pub"].publish(Bool(data=self.is_tele_enabled))
                else:
                    if self.enable_tele:
                        # 텔레오퍼레이션 모드가 활성화되면 
                        self.Tmh_ref = None
                        self.target_offset_last = np.copy(self.target_offset)
                        self.is_tele_enabled = self.enable_tele
                        self.robot_pubs[self.current_robot]["finger_pub"].publish(Bool(data=self.is_tele_enabled))
                    else:
                        # The held self.target_offset is repeated by timer_callback 텔레오퍼레이션 모드가 아니면 이전 목표 위치를 유지
                        pass

        else: # 텔레오퍼레이션 모드가 아닌 경우:
            # 목표 위치 및 회전 정보 초기화
            self.target_offset = [0]*6
            self.target_offset_last = [0]*6

        if self.save_log: # 로그 저장버튼 on
            self.elapsed_time = (time.time_ns() - self.start_time)/ 1e9
            self.sigma_log.append([self.elapsed_time] + list(self.Pmh) + list(Rotation.from_dcm(self.Rmh).as_rotvec()))
    '''
    Sigma publish
    '''
    def timer_callback(self):
        self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
        if self.tele_mode: # 텔레오퍼레이션 모드인 경우
              # 목표 위치 및 회전 정보를 메시지에 담아서 발행 (publish)합니다.
            self.tele_move_msg.data = list(map(float, self.target_offset))#
            self.robot_pubs[self.current_robot]["tele_move_pub"].publish(self.tele_move_msg)

        if self.save_log:  # 로그 저장버튼 On, 현재 시간과 함께 목표 위치 및 회전 정보를 로그에 기록합니다.
            self.elapsed_time = (time.time_ns() - self.start_time)/ 1e9
            self.tele_move_log.append([self.elapsed_time] + list(map(float, self.target_offset)))
        else: # 로그 저장버튼 Off, 로그 데이터를 저장합니다. (if not self.save_log: )
            if len(self.tele_move_log) > 0:
                np.savetxt(os.path.join(TMP_LOG_PATH, f"TELE_MOVE_LOG_{self.current_robot}_{self.current_datetime}.csv"), self.tele_move_log, delimiter=",",header=",".join(self.tele_move_labels), comments="")
                self.tele_move_log = []
            if len(self.sigma_log) > 0:
                np.savetxt(os.path.join(TMP_LOG_PATH, f"SIGMA_LOG_{self.sigma_name}_{self.current_datetime}.csv"), self.sigma_log, delimiter=",", header=",".join(self.sigma_labels), comments="")
                self.sigma_log = []


def main(args=None):
    rclpy.init(args=args)
    gamepad_service_client = IndySigmaConnection()
    rclpy.spin(gamepad_service_client)
    gamepad_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
