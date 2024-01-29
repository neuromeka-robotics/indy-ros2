#!/usr/bin/python3
#-*- coding: utf-8 -*-
# import sys
import json
from copy import deepcopy

import math
import time
from functools import partial
from threading import Thread

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool, Float32MultiArray, Float32
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from ament_index_python.packages import get_package_share_directory

from datetime import datetime
from collections import deque

import sys
import os

__client_path = os.path.join(get_package_share_directory('indy_driver'), "client_wrapper")
__grpc_path = os.path.join(get_package_share_directory('indy_driver'), "gRPCServerGenPython")
__nrmk_path = os.path.join(get_package_share_directory('indy_driver'), "nrmk_utils")

sys.path.append(__client_path)
sys.path.append(__grpc_path)
sys.path.append(__nrmk_path)
from indy_grpc_client import *
from math_tools import *
from indy_driver import *
from file_io import *

TMP_LOG_PATH = os.path.join(get_home_path(), "TMP_LOG")
if not os.path.isdir(TMP_LOG_PATH):
    os.mkdir(TMP_LOG_PATH)


##
# @class BlockWrapper
# @brief a enter/exit function
class BlockWrapper:
    def __init__(self, enter_fn, exit_fn):
        self.enter_fn, self.exit_fn = enter_fn, exit_fn

    def __enter__(self):
        self.enter_fn()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.exit_fn()


class IndyROSConnector(Node):

    def __init__(self):
        super().__init__('indy_driver')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        # Initialize parameters  with default values
        self.declare_parameter('robot_name', "robot0")
        self.declare_parameter('robot_ip', "127.0.0.1")
        self.declare_parameter('robot_type', "indy7")
        self.declare_parameter('home_pose', [0.0, 0.0, -1.57, 0.0, -1.57, 0.0])
        self.declare_parameter('origin.xyz', [0.0,0.0,0.0])
        self.declare_parameter('origin.rpy', [0.0,0.0,0.0])
        self.declare_parameter('tcp', [0.0,0.0,0.0])
        self.declare_parameter('pub_rate', 20.0)
        self.declare_parameter('gripper_di', 0)
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value + "_"
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.home_pose = self.get_parameter('home_pose').get_parameter_value().double_array_value
        self.origin_xyz = self.get_parameter('origin.xyz').get_parameter_value().double_array_value
        self.origin_rpy = self.get_parameter('origin.rpy').get_parameter_value().double_array_value
        self.tcp = self.get_parameter('tcp').get_parameter_value().double_array_value
        self.pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value
        self.gripper_di = self.get_parameter('gripper_di').get_parameter_value().integer_value
        self.gripper_state = False
        self.enable_tele = False
        self.save_log = False
        self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
        self.tele_indy_labels = ["Time", "X", "Y", "Z", "U","V","W"] # Add labels for tele_indy_log columns
        self.callback_labels = ["fn_id", "Callback Time", "Current Time"] # Add labels for callback_log columns
        self.joint_indy_labels = ["Time", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" ]
        self.joint_vel_indy_labels = ["Time", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" ]
        self.teaching_label = ["Time", "TX", "TY", "TZ", "TU","TV", "TW","X", "Y", "Z", "U","V", "W","dX", "dY", "dZ", "dU","dV","dW","Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" ,"dJoint1", "dJoint2", "dJoint3", "dJoint4", "dJoint5", "dJoint6"]


        self.get_logger().info(f"ROBOT IP: {self.robot_ip}")
        self.get_logger().info(f"ROBOT TYPE: {self.robot_type}")
        self.get_logger().info(f"ORIGIN: {self.origin_xyz} / {self.origin_rpy}")
        self.get_logger().info(f"PUBLISH RATE: {self.pub_rate} Hz")

        self.task_cmd = [0]*6
        self.Rgr = Rot_rpy(*self.origin_rpy)        # global-robot R self.origin_rpy에 저장된 Roll, Pitch, Yaw (RPY) 회전 각도를 사용하여 로봇의 전역 좌표계에서 로봇의 회전을 나타내는 회전 행렬 self.Rgr을 계산합니다.
        self.Rrg = self.Rgr.transpose()             # robot-global R 로봇-전역 좌표계 간의 회전 행렬을 계산하기 위해 self.Rgr의 전치 행렬을 구합니다. 이것은 로봇의 회전을 전역 좌표계에서 로봇 좌표계로 변환합니다.
        self.Tgr = SE3(self.Rgr, self.origin_xyz)   # global-robot T 로봇의 회전 행렬 self.Rgr과 로봇의 원점 좌표 self.origin_xyz를 사용하여 로봇의 전역 좌표계에서 로봇의 위치와 자세를 나타내는 변환 행렬 self.Tgr을 계산합니다.
        self.Trg = np.linalg.inv(self.Tgr)          # robot-global T self.Tgr의 역행렬을 계산하여 로봇 좌표계에서 전역 좌표계로의 변환 행렬 self.Trg를 얻습니다. 이것은 로봇 좌표계에서의 위치와 자세를 전역 좌표계로 변환합니다.
        self.Tet = SE3(np.identity(3), self.tcp)    # 로봇의 도구(TCP, Tool Center Point) 좌표를 나타내는 벡터 self.tcp와 단위 행렬을 사용하여 로봇 도구 좌표계에서의 변환 행렬 self.Tet을 계산합니다.
        self.Tte = np.linalg.inv(self.Tet)          # self.Tet의 역행렬을 계산하여 도구 좌표계에서 로봇 좌표계로의 변환 행렬 self.Tte를 얻습니다. 이것은 도구 좌표계에서의 위치와 자세를 로봇 좌표계로 변환합니다.

        self.start_time = time.time_ns()
        self.elapsed_time = 0
        self.is_connected = False
        self.task_indy_log = []
        self.task_vel_indy_log = []
        self.joint_indy_log = []
        self.joint_vel_indy_log = []
        self.tele_indy_log = []
        self.teaching_log=[]
        self.callback_log = []
        self.callback_time_dict = {}
        self.target_offset = [0]*6
        self.__di = [False]*100
        self.__q = [0]*20
        self.__p = [0]*20
        self.__qdot = [0]*20
        self.__pdot = [0]*20
        self.pre_tau_value = [0]*20
        self.__torque = [0]*20
        self.__current = [0]*20

        
        self.recording = False
        self.recording_check = True
        self.playing = False
        self.playing_check = True
        self.task_motion_list = [0]*7
        self.pre_time = 0
        self.cur_time = 0
        if self.robot_type=='indyrp2' or self.robot_type=='indyrp2_v2':
            self.task_pos = [0]*7
            self.record_start_joint_position = [0]*7
        else:
            self.task_pos = [0]*6
            self.record_start_joint_position = [0]*6
    

        # Initialize joint control servers
        self.jtc_action_server = ActionServer(  #로봇의 관절을 제어하기 위한 FollowJointTrajectory 액션 서버를 생성, 로봇 제어 시스템에서 액션 클라이언트로부터 받은 액션 목표(goal)를 수신하고, 실행(execute) 및 취소(cancel)를 관리하는 데 사용
            self,
            FollowJointTrajectory,
            "/" + robot_name + 'joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        
        self.joint_position_sub = self.create_subscription( # 로봇의 관절 움직임 
            JointTrajectory,
            robot_name + 'joint_trajectory',
            self.joint_position_callback,
            qos_profile)

        # Initialize topics
        self.timer = self.create_timer(1/self.pub_rate, self.timer_callback)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            robot_name + "joint0",
            robot_name + "joint1",
            robot_name + "joint2",
            robot_name + "joint3",
            robot_name + "joint4",
            robot_name + "joint5"
        ]
        if self.robot_type == 'indyrp2' or self.robot_type == 'indyrp2_v2':
            self.joint_state_msg.name.append(robot_name + "joint6")

        self.task_state_pub = self.create_publisher(Float32MultiArray, robot_name + 'task_states', qos_profile)
        self.task_state_msg = Float32MultiArray()

        self.each_joint_state_pub = self.create_publisher(Float32MultiArray, robot_name + 'joint_states', qos_profile)
        self.each_joint_state_msg = Float32MultiArray()

        self.current_state_pub = self.create_publisher(Float32MultiArray, robot_name + 'current_states', qos_profile)
        self.current_state_msg = Float32MultiArray()
        

        self.manual_sub = self.create_subscription(
            Float32MultiArray,
            robot_name + 'manual_motion',
            self.manual_callback,
            qos_profile)

        self.save_log_sub = self.create_subscription(
            Bool,
            'save_log_status',
            self.save_log_callback,
            qos_profile)
        
        self.record_sub = self.create_subscription(
            Bool,
            robot_name + 'record_status',
            self.record_callback,
            qos_profile)

        self.play_sub = self.create_subscription(
            Bool,
            robot_name + 'play_status',
            self.play_callback,
            qos_profile)


        self.tele_mode_sub = self.create_subscription(
            Bool, 
            robot_name + 'tele_mode', 
            self.tele_mode_callback, 
            qos_profile)

        self.tele_move_sub = self.create_subscription(
            Float32MultiArray, 
            robot_name + 'tele_move', 
            self.tele_move_callback, 
            qos_profile)
        
        self.finger_sub = self.create_subscription(
            Bool,
            robot_name + "tele_finger",
            self.finger_callback,
            qos_profile)

        self.reset_sub = self.create_subscription(
            Bool,
            robot_name + "reset",
            self.reset_callback,
            qos_profile)

        self.home_sub = self.create_subscription(
            Bool,
            robot_name + "home",
            self.home_callback,
            qos_profile)


        self.error_pub = self.create_publisher(Bool, robot_name + 'error', qos_profile) # robot의 error상태 발생 

        # Initialize variable
        self.vel = 3 # level 1 -> 3
        # self.blend = 0.2 # rad 0 -> 0.4
        self.joint_state_list = []
        self.joint_state_feedback = JointTrajectoryPoint()
        self.joint_ref = None
        self.wait_init_tele_mode = False
        self.tele_mode_on = False
        self.monitor_thread = Thread(target=self.loop_indy_monitor, daemon=True)
        self.monitor_thread.start()
        print("Indy connector has been initialised.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    def loop_indy_monitor(self):
        
        while True:
            try:
                if self.is_connected:
                    self.__di = self.indy.get_di()
                    self.__q = self.indy.get_joint_pos()
                    self.__qdot = self.indy.get_joint_vel()
                    self.__p = self.indy.get_task_pos()
                    self.__pdot = self.indy.get_task_vel()
                    self.__torque = self.indy.get_control_torque()

                    if self.__torque.count(0) > 0: # 
                        pass
                    else: 
                        if self.robot_type == 'indyrp2' or self.robot_type == 'indyrp2_v2':
                            factors = [4.987, 5.249, 9.973, 9.973, 18.215, 19.174, 18.215]
                        elif self.robot_type == 'indy12' or self.robot_type == 'indy12_v2':
                            factors = [1.034, 1.093, 5.249, 10.137, 10.137, 9.123]
                        elif self.robot_type == 'indy7' or self.robot_type == 'indy7_v2':
                            factors = [4.488, 4.724, 8.900, 16.393, 17.256, 16.393]
                        result = [x * factor for x, factor in zip(self.__torque, factors)]
                        self.__current = result                        

                    err = self.indy.get_robot_status()['violation']
                    self.error_pub.publish(Bool(data=bool(err)))
                time.sleep(1/self.pub_rate)
            except Exception as e:
                self.get_logger().error("Error in indy monitor loop")
                self.get_logger().error(str(e))

    def __enter_timecheck(self, fn_id):
        if self.save_log:
            self.callback_time_dict[fn_id] = time.time()

    def __exit_timecheck(self, fn_id):
        if self.save_log:
            self.callback_log.append([fn_id, self.callback_time_dict[fn_id], time.time()])

    '''
    Connecting to Indy
    '''
    # Connect to Indy
    def connect(self):
        self.indy = IndyMaster(self.robot_ip)
        self.is_connected = True
        self.indy.set_joint_vel_level(self.vel)
        self.indy.write_direct_variable(0,0,0) #Gripper Speed
        time.sleep(0.5)
        self.indy.write_direct_variable(0,0,1) #Gripper Force
        time.sleep(0.5)
        self.indy.write_direct_variable(0,0,9) #Gripper Initialization
        self.indy.write_direct_variable(0,2,100) #Gripper speed
        self.indy.write_direct_variable(0,3,50) #Gripper force
        self.indy.write_direct_variable(0,4,2) #Gripper actual Position read mode
        time.sleep(0.3)

    # Disconnect to Indy
    def disconnect(self):
        self.is_connected = False
        self.stop_motion()
        self.get_logger().warning("DISCONNECT TO ROBOT")
        time.sleep(0.3)
        self.indy.channel.close()

    def get_indy_error_msg(self):
        state_dict = self.indy.get_robot_status()
        if state_dict['busy']:
            return "ROBOT IS BUSY"
        if state_dict['resetting']:
            return "ROBOT IS IN RECOVER STATE"
        if state_dict['collision']:
            return "ROBOT IS IN COLLISION STATE"
        if state_dict['teaching']:
            return "ROBOT IS IN TEACHING MODE"
        if state_dict['violation']:
            return "ROBOT IS IN VIOLATION STATE"
        return None

    def joint_position_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 1), #코드 실행 시간을 측정하고 로깅을 수행
                          partial(self.__exit_timecheck, 1)):
            self.stop_motion()
            #관절 움직임 목록 추출
            joint_motion_list = [p.positions for p in msg.points]

            #--------------reduce the trajectory points(경로 최적화)----------
            # new_array = joint_motion_list
            # # self.get_logger().info(f'empty new array = {new_array}')
            # percentage = 50 # percentage of reduced point
            # number = int(len(joint_motion_list) * (1 - (percentage/100)))# 151개 -> number = 75
            # # self.get_logger().info(f'number = {number}')
            # if number > 0:
            #     distance = int((len(joint_motion_list)) / (number)) #distance = 151/75 = 2
            #     for i in range(1, number + 1):                              #1 부터 75까지 반복
            #         new_array.append(joint_motion_list[(i * distance)-1])       # new_array에 50개만 담는다? distance만 띄어서
            # new_array.append(joint_motion_list[-1])
            # # self.get_logger().info(f'new_array = {new_array}')

            #---------------------------------------------------
            jcount = len(joint_motion_list)

            # 각 웨이포인트로 이동하면서 로봇을 제어합니다.
            self.start_joint_motion()
            radii = np.deg2rad(20)

            # for joint_rads in joint_motion_list: # 관전 목록에 있는 각 관절 위치로 로봇을 움직임
            for i_j, joint_rads in enumerate(joint_motion_list):
                # 마지막 웨이포인트의 반경은 작게 설정하고, 그 외에는 크게 설정합니다. -> 부드러운 로봇 움직임으로 안정적으로 동작
                radii = np.deg2rad(6) if i_j >= (jcount-1) else np.deg2rad(20)
                joint_rads = list(map(float, joint_rads))
                self.move_joint(joint_rads)
                while np.sum(np.abs(deg2rad(self.indy.get_joint_pos()) - joint_rads)) > radii:
                    time.sleep(0.01)

    # 텔레오퍼레이션 모드로 이동할 때 호출되는 콜백 함수
    def tele_move_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 2),
                          partial(self.__exit_timecheck, 2)):
            
            # 초기 텔레오퍼레이션 모드 대기 중인 경우(True)
            if self.wait_init_tele_mode:
                # 로봇의 오류 메시지를 확인
                error = self.get_indy_error_msg()
                if error is not None:
                    # 오류가 발생한 경우 경고 메시지를 출력
                    self.get_logger().warning(error + 'WHILE TELE-MOVING')
                else:
                    # 오류가 없으면 초기 텔레오퍼레이션 모드를 종료하고 텔레오퍼레이션 모드를 시작합니다.
                    self.wait_init_tele_mode = False
                    self.start_teleop()
                    self.get_logger().warning(f"Turn ON Tele Mode DONE For {self.robot_ip}")
                return
            
            if self.recording:
                if self.recording_check:
                    temp_bool_msg = Bool()
                    temp_bool_msg.data = True
                    self.tele_mode_callback(temp_bool_msg)
                    self.record_start_joint_position = self.indy.get_joint_pos()
                    self.task_motion_list = []*7
                    self.get_logger().warn(f"Recording Start {self.robot_ip}")
                    self.recording_check = False

                else: 
                    self.task_motion_list.append([time.time()] + msg.data.tolist())
            else:
                self.recording_check = True
            self.update_teleop_target(msg.data)
            self.target_offset = msg.data
            
            if self.save_log: # 로그 저장버튼 On
                self.elapsed_time = (time.time_ns() - self.start_time)/ 1e9
                self.tele_indy_log.append([self.elapsed_time] + list(msg.data))
                

    def start_teleop(self):
        task_cmd = self.indy.get_task_pos()  # m, degs
        self.Tre_ref = cmd2T(task_cmd)
        self.Trt_ref = np.matmul(self.Tre_ref, self.Tet)
        self.Tle = SE3(np.identity(3), self.Tre_ref[:3,3])  # 엔드이펙터 로컬 좌표계 (로봇 방향 + 엔드이펙터 위치)
        self.Tlt = SE3(np.identity(3), self.Trt_ref[:3,3])  # 툴 로컬 좌표계 (로봇 방향 + 툴 위치)
        self.Tet = np.matmul(np.linalg.inv(self.Tle), self.Tlt)
        self.Tte = np.linalg.inv(self.Tet)
        self.indy.set_sync_mode(False)
        self.indy.start_teleoperation()
        self.tele_mode_on = True

    ##
    # @param task_cmd   xyz (m), rotvec (rads)
    def update_teleop_target(self, xyzrvec):
        with BlockWrapper(partial(self.__enter_timecheck, 31),
                          partial(self.__exit_timecheck, 31)):
            
            # 입력 매개변수에서 원하는 위치(x, y, z 좌표)와 회전(rvec) 벡터를 추출합니다.
            xyz_g = xyzrvec[:3]  # 글로벌 좌표계 기준 툴 변위
            rvec_g = xyzrvec[3:]  # 글로벌 좌표계 기준 툴 회전 벡터

            # 툴의 글로벌 변위 및 회전 벡터를 툴 로컬 좌표계(로봇 방향 + 툴 위치)에 맞추기 위해 로봇 방향으로 회전
            xyz_t = np.matmul(self.Rrg, xyz_g)  # 로봇 방향 기준 툴 위치 변위
            rvec_t = np.matmul(self.Rrg, rvec_g)  # 로봇 방향 기준 툴 회전 벡터

            # dTt는 기준 툴 좌표계와 이동된 툴 좌표계 간 변환 행렬 (Tt<-t2)
            dTt = SE3(Rotation.from_rotvec(rvec_t).as_dcm(), xyz_t)

            # 툴 로컬 좌표계 변환 행렬을 엔드이펙터 로컬 좌표계 변환 행렬로 전환
            # T(e<-e2) = T(e<-t)*T(t<-t2)*T(t2<-e2) = Tet*dTt*Tte
            dTe = np.matmul(self.Tet, np.matmul(dTt, self.Tte))

            # 엔드이펙터 로컬 좌표계의 변위 및 회전 벡터를 추출
            xyz_e = dTe[:3,3]  # 엔드이펙터 로컬 좌표계 기준, 새 엔드이펙터의 위치
            rvec_e = Rotation.from_dcm(dTe[:3,:3]).as_rotvec()  # 엔드이펙터 로컬 좌표계 기준, 새 엔드이펙터 방향으로의 회전 벡터

            # 업데이트된 위치와 회전 정보(xyz_e 및 rvec_e)를 사용하여 로봇의 텔레오퍼레이션 궤적을 업데이트합니다.
            # update_teleoperation_traj는 엔드이펙터 로컬 좌표계(로봇 방향 + 엔드이펙터 위치)의 변위 및 회전 벡터를 받습니다.
            self.indy.update_teleoperation_traj(np.concatenate([xyz_e, rvec_e]).tolist(), -1)

    def start_joint_motion(self):
        if self.joint_ref is None:
            self.joint_ref = deg2rad(self.indy.get_joint_pos())
        self.indy.set_sync_mode(True)
        self.indy.start_teleoperation_joint()

    def move_joint(self, joint_rads):
        target_joint_pos = np.subtract(joint_rads, self.joint_ref).tolist()
        self.indy.update_teleoperation_traj_joint(
            target_joint_pos=target_joint_pos, time_const=-1
        )

    def stop_motion(self):
        self.get_logger().info("STOP CALLED")
        self.indy.stop_teleoperation()
        self.indy.stop_teleoperation_joint()
        self.indy.update_teleoperation_traj([0]*6, -1)
        self.indy.update_teleoperation_traj_joint([0]*6, -1)
        self.joint_ref = None
        self.indy.stop_motion()
        self.tele_mode_on = False

    def manual_callback(self, msg):
        target_p = msg.data.tolist()
        init_q = self.indy.get_joint_pos()
        inv_kin = self.indy.get_inv_kin(target_p, init_q)
        if all(value == 0 for value in inv_kin):
            self.get_logger().warn("Can not Slove the Kinematics. Please check the target position")
        else:
            self.indy.set_sync_mode(True)
            self.indy.joint_move_to(inv_kin)

    def record_callback(self, msg):
        if msg.data is True:
            self.recording = True
        elif msg.data is False:
            self.recording = False

    def play_callback(self, msg):
        if msg.data is True:
            self.playing = True
            self.playing_check = True
        elif msg.data is False:
            self.playing = False     

    def save_log_callback(self, msg):
        if msg.data is True:
            self.save_log = True
        elif msg.data is False:
            self.save_log = False

    def tele_mode_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 3),
                          partial(self.__exit_timecheck, 3)):
            if msg.data is True:
                self.stop_motion()
                # self.indy.joint_move_to(rad2deg(self.home_pose).tolist())
                time.sleep(0.1)
                self.wait_init_tele_mode = True
            elif msg.data is False:
                self.stop_motion()

    def finger_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 4),
                          partial(self.__exit_timecheck, 4)):
            self.enable_tele = msg.data
            di_list = self.__di.copy()
            if di_list[self.gripper_di] != self.gripper_state:
                self.gripper_state = di_list[self.gripper_di]
                if self.gripper_state:
                    self.indy.write_direct_variable(0,1,85) #Gripper Close
                    self.get_logger().info('Gripper close')
                else:
                    self.indy.write_direct_variable(0,1,0) #Gripper Close
                    self.get_logger().info('Gripper open')
    
    def home_callback(self,msg):
        if self.indy.get_robot_status()['busy'] == 0:
            self.indy.go_home()

    def reset_callback(self, msg):
        self.indy.reset_robot()

    '''
    Indy publish
    '''
    # Publish jointstates and taskstates
    def position_state_publisher(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = deg2rad(self.__q).tolist()
        # joint_state_msg.velocity = self.indy.get_joint_vel() TODO
        # joint_state_msg.effort = get_control_torque() TODO
        self.joint_state_feedback.positions = self.joint_state_msg.position
        self.joint_state_pub.publish(self.joint_state_msg)
        
        self.each_joint_state_msg.data = self.__q# x y z u v w (uint : meter , degree)
        self.each_joint_state_pub.publish(self.each_joint_state_msg)

        self.task_state_msg.data = self.__p# x y z u v w (uint : meter , degree)
        self.task_state_pub.publish(self.task_state_msg)

        self.current_state_msg.data = self.__current# x y z u v w (uint : meter , degree)
        self.current_state_pub.publish(self.current_state_msg)
        
        if self.save_log:  # 로그 저장버튼 On
            self.elapsed_time = (time.time_ns() - self.start_time)/ 1e9
            self.task_indy_log.append(deepcopy([self.elapsed_time] + self.__p)) # task 로그
            self.task_vel_indy_log.append(deepcopy([self.elapsed_time] + self.__pdot))
            self.joint_indy_log.append(deepcopy([self.elapsed_time] + self.__q)) # joint 로그
            self.joint_vel_indy_log.append(deepcopy([self.elapsed_time] + self.__qdot))
            self.teaching_log.append(deepcopy([self.elapsed_time] + list(self.target_offset) + self.__p + self.__pdot + self.__q + self.__qdot))

    # Timer callback for publish
    def timer_callback(self):
        with BlockWrapper(partial(self.__enter_timecheck, 5),
                          partial(self.__exit_timecheck, 5)):
            self.position_state_publisher()
            if self.enable_tele:
                di_list = self.__di.copy()
                if di_list[self.gripper_di] != self.gripper_state:
                    self.gripper_state = di_list[self.gripper_di]
                    self.get_logger().info('Gripper to do')
                    if self.gripper_state == True:
                        self.indy.write_direct_variable(0,1,85) #Gripper Close
                        self.get_logger().info('Gripper close')
                    else:
                        self.indy.write_direct_variable(0,1,0) #Gripper Close
                        self.get_logger().info('Gripper open')
               
        if self.playing:
            if self.playing_check:
                self.playing_check = False
                temp_bool_msg = Bool()
                temp_bool_msg.data = True
                self.tele_mode_callback(temp_bool_msg)
                if len(self.task_motion_list) > 10: #녹화된 모션이 있으면 
                    # 초기 텔레오퍼레이션 모드 대기 중인 경우(True)
                    if self.wait_init_tele_mode:
                        # 로봇의 오류 메시지를 확인
                        error = self.get_indy_error_msg()
                        if error is not None:
                            # 오류가 발생한 경우 경고 메시지를 출력
                            self.get_logger().warning(error + 'WHILE TELE-MOVING')
                        else:
                            # 오류가 없으면 초기 텔레오퍼레이션 모드를 종료하고 텔레오퍼레이션 모드를 시작합니다.
                            self.indy.set_sync_mode(True)
                            self.indy.joint_move_to(self.record_start_joint_position)
                            print(f"현재값 : {self.indy.get_joint_pos()}")
                            print(f"초기값 : {self.record_start_joint_position}")
                            while np.sum(np.abs(np.array(self.indy.get_joint_pos()) - np.array(self.record_start_joint_position))) > 0.1:
                                time.sleep(0.01)
                            self.wait_init_tele_mode = False
                            self.start_teleop()
                            self.get_logger().warning(f"Playing recorded motion {self.robot_ip}")   
                                               

                    # for joint_rads in joint_motion_list: # 관전 목록에 있는 각 관절 위치로 로봇을 움직임
                    i = 0
                    self.pre_time = self.task_motion_list[0][0]
                    for sublist in self.task_motion_list:
                        i+=1
                        self.cur_time = sublist[0]
                        self.task_pos = sublist[1:]
                        # 마지막 웨이포인트의 반경은 작게 설정하고, 그 외에는 크게 설정합니다. -> 부드러운 로봇 움직임으로 안정적으로 동작
                        self.task_pos = list(map(float, self.task_pos))
                        self.update_teleop_target(self.task_pos)
                        while np.sum(np.abs(np.array(self.indy.get_task_pos()) - np.array(self.task_pos))) > 0.1:
                            time.sleep(self.cur_time-self.pre_time)
                            break
                        self.pre_time = self.cur_time
                    self.stop_motion()
                    self.get_logger().warning(f"Playing fnished {self.robot_ip}")  
                

        if not self.save_log: #로그 저장버튼 Off
            if len(self.tele_indy_log)>0:
                self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"TELE_INDY_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.tele_indy_log, delimiter=",", header=",".join(self.tele_indy_labels), comments="")
                self.tele_indy_log = []
            if len(self.callback_log)>0:
                self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"CALLBACK_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.callback_log, delimiter=",", header=",".join(self.callback_labels), comments="")
                self.callback_log = []
            if len(self.joint_indy_log)>0:
                self.current_datetime = datetime.now().strftime("%Y%m%d%H%M%S")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"TASK_INDY_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.task_indy_log, delimiter=",",header=",".join(self.tele_indy_labels), comments="")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"TASK_VEL_INDY_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.task_vel_indy_log, delimiter=",",header=",".join(self.tele_indy_labels), comments="")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"JOINT_INDY_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.joint_indy_log, delimiter=",",header=",".join(self.joint_indy_labels), comments="")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"JOINT_VEL_INDY_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.joint_vel_indy_log, delimiter=",",header=",".join(self.joint_vel_indy_labels), comments="")
                np.savetxt(os.path.join(TMP_LOG_PATH, f"TEACHING_LOG_{self.robot_ip}_{self.current_datetime}.csv"), self.teaching_log, delimiter=",",header=",".join(self.teaching_label), comments="")
                self.joint_indy_log = []
                self.joint_vel_indy_log = []


    '''
    Indy follow joint trajectory 
    '''
    def goal_callback(self, goal_request):
        with BlockWrapper(partial(self.__enter_timecheck, 6),
                          partial(self.__exit_timecheck, 6)):
            # Accepts or rejects a client request to begin an action
            self.get_logger().info('Received goal request :)')
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        with BlockWrapper(partial(self.__enter_timecheck, 7),
                          partial(self.__exit_timecheck, 7)):
            # Accepts or rejects a client request to cancel an action
            self.get_logger().info('Received cancel request :(')
            return CancelResponse.ACCEPT
        
    #FollowJointTrajectory 액션 서버의 실행 콜백 함수
    async def execute_callback(self, goal_handle):
        with BlockWrapper(partial(self.__enter_timecheck, 8),
                          partial(self.__exit_timecheck, 8)):
            print('FollowJointTrajectory callback...')
            # last_time = self.get_clock().now()
            goal = goal_handle.request.trajectory.points.copy()

            # download planned path from ros moveit
            self.joint_state_list = []
            if goal:
                self.joint_state_list = [p.positions for p in goal]
            else:
                self.stop_motion()

            # Do something for OP_IDLE state
            if self.joint_state_list:

                result = FollowJointTrajectory.Result()
                feedback_msg = FollowJointTrajectory.Feedback()
                # 로봇 관절 상태의 오류를 확인
                error = self.get_indy_error_msg()
                if error is not None:
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    result.error_string = error
                    return result
                # 로봇의 관절 움직임을 시작합니다.
                self.start_joint_motion()
                jcount = len(self.joint_state_list)
                self.get_logger().info(f'Follow {jcount} waypoints')

                # 각 웨이포인트로 이동하면서 로봇을 제어합니다.
                for i_j, joint_rads in enumerate(self.joint_state_list):
                    # 마지막 웨이포인트의 반경은 작게 설정하고, 그 외에는 크게 설정합니다. -> 부드러운 로봇 움직임으로 안정적으로 동작
                    radii = np.deg2rad(6) if i_j >= (jcount-1) else np.deg2rad(20)
                    joint_rads = list(map(float, joint_rads))
                    self.move_joint(joint_rads)
                    self.get_logger().info('robot is moving...')
                    # 로봇이 목표 관절 위치에 도달할 때까지 대기
                    while np.sum(np.abs(deg2rad(self.indy.get_joint_pos()) - joint_rads)) > radii:
                        if goal_handle.is_cancel_requested:
                            self.stop_motion()
                            goal_handle.canceled()
                            return result

                        # print("self.joint_state_feedback.positions: ", self.joint_state_feedback.positions)
                        # 피드백 메시지를 설정하고 발행
                        feedback_msg.desired.positions = self.joint_state_feedback.positions
                        feedback_msg.actual.positions = self.joint_state_feedback.positions
                        goal_handle.publish_feedback(feedback_msg)

                        # self.get_logger().info('robot is moving...')
                        time.sleep(0.1)
                # 로봇 움직임을 중지        
                self.stop_motion()
            # 액션을 성공으로 표시하고 결과를 설정
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result


def main(args=None):
    rclpy.init(args=args)

    with IndyROSConnector() as indy_driver:
        executor = MultiThreadedExecutor()
        rclpy.spin(indy_driver, executor=executor) # ROS 2 노드(Node)를 실행하고, 노드 내에 등록된 콜백 함수들을 실행합니다. 

    rclpy.shutdown()


if __name__ == '__main__':
    main()
