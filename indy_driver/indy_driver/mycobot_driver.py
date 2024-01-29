import rclpy
import time
from datetime import datetime
from functools import partial
from pymycobot.mycobot import MyCobot
# from pymycobot.mycobotsocket import MyCobotSocket
import RPi.GPIO as GPIO

import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.time import Time, Duration
from std_msgs.msg import String, Bool, Float32MultiArray, Float32, Int32
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from ament_index_python.packages import get_package_share_directory
import math

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
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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



class Sigma_MyCobot_Comm(Node):

    MAX_LEN = 280 # mm
    MAX_ANGLE = 270 # deg
    # SPEED = 50
    MODE = 0

    LEN_TO_UPDATE = 5 # mm
    ANGLE_TO_UPDATE = 3 # deg

    def __init__(self):
        super().__init__("control_slider")
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
        # self.declare_parameter('gripper_di', 0)
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value + "_"
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.home_pose = self.get_parameter('home_pose').get_parameter_value().double_array_value
        self.origin_xyz = self.get_parameter('origin.xyz').get_parameter_value().double_array_value
        self.origin_rpy = self.get_parameter('origin.rpy').get_parameter_value().double_array_value
        self.tcp = self.get_parameter('tcp').get_parameter_value().double_array_value
        self.pub_rate = self.get_parameter('pub_rate').get_parameter_value().double_value
        # self.gripper_di = self.get_parameter('gripper_di').get_parameter_value().integer_value
        self.gripper_state = False
        self.enable_tele = False
        self.save_log = False
        self.current_datetime = datetime.now().strftime("%Y%m%d%H%M")
        # self.tele_indy_labels = ["Time", "X", "Y", "Z", "U","V","W"] # Add labels for tele_indy_log columns
        # self.callback_labels = ["fn_id", "Callback Time", "Current Time"] # Add labels for callback_log columns
        # self.joint_indy_labels = ["Time", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" ]
        # self.joint_vel_indy_labels = ["Time", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6" ]

        self.Rgr = Rot_rpy(*self.origin_rpy)        # global-robot R self.origin_rpy에 저장된 Roll, Pitch, Yaw (RPY) 회전 각도를 사용하여 로봇의 전역 좌표계에서 로봇의 회전을 나타내는 회전 행렬 self.Rgr을 계산합니다.
        self.Rrg = self.Rgr.transpose()             # robot-global R 로봇-전역 좌표계 간의 회전 행렬을 계산하기 위해 self.Rgr의 전치 행렬을 구합니다. 이것은 로봇의 회전을 전역 좌표계에서 로봇 좌표계로 변환합니다.
        self.Tgr = SE3(self.Rgr, self.origin_xyz)   # global-robot T 로봇의 회전 행렬 self.Rgr과 로봇의 원점 좌표 self.origin_xyz를 사용하여 로봇의 전역 좌표계에서 로봇의 위치와 자세를 나타내는 변환 행렬 self.Tgr을 계산합니다.
        self.Trg = np.linalg.inv(self.Tgr)          # robot-global T self.Tgr의 역행렬을 계산하여 로봇 좌표계에서 전역 좌표계로의 변환 행렬 self.Trg를 얻습니다. 이것은 로봇 좌표계에서의 위치와 자세를 전역 좌표계로 변환합니다.
        self.Tet = SE3(np.identity(3), self.tcp)    # 로봇의 도구(TCP, Tool Center Point) 좌표를 나타내는 벡터 self.tcp와 단위 행렬을 사용하여 로봇 도구 좌표계에서의 변환 행렬 self.Tet을 계산합니다.
        self.Tte = np.linalg.inv(self.Tet)          # self.Tet의 역행렬을 계산하여 도구 좌표계에서 로봇 좌표계로의 변환 행렬 self.Tte를 얻습니다. 이것은 도구 좌표계에서의 위치와 자세를 로봇 좌표계로 변환합니다.

        self.get_logger().info(f"ROBOT IP: {self.robot_ip}")
        self.get_logger().info(f"ROBOT TYPE: {self.robot_type}")
        self.get_logger().info(f"ORIGIN: {self.origin_xyz} / {self.origin_rpy}")
        self.get_logger().info(f"PUBLISH RATE: {self.pub_rate} Hz")

        self.__di = [False]*100
        self.__q = [0]*20

        self.speed = 50


        # self.jtc_action_server = ActionServer(  #로봇의 관절을 제어하기 위한 FollowJointTrajectory 액션 서버를 생성, 로봇 제어 시스템에서 액션 클라이언트로부터 받은 액션 목표(goal)를 수신하고, 실행(execute) 및 취소(cancel)를 관리하는 데 사용
        #     self,
        #     FollowJointTrajectory,
        #     "/" + robot_name + 'joint_trajectory_controller/follow_joint_trajectory',
        #     execute_callback=self.execute_callback,
        #     goal_callback=self.goal_callback,
        #     cancel_callback=self.cancel_callback,
        #     )
        # self.joint_position_sub = self.create_subscription( # 로봇의 관절 움직임
        #     JointTrajectory,
        #     robot_name + 'joint_trajectory',
        #     self.joint_position_callback,
        #     qos_profile)

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
        self.error_pub = self.create_publisher(Int32, robot_name + 'error', qos_profile) # robot의 error상태 발생


        # Initialize variable
        # self.vel = 3 # level 1 -> 3
        # self.blend = 0.2 # rad 0 -> 0.4
        self.joint_state_list = []
        self.joint_state_feedback = JointTrajectoryPoint()
        self.joint_ref = None
        self.wait_init_tele_mode = False
        self.tele_mode_on = False
        # self.monitor_thread = Thread(target=self.loop_indy_monitor, daemon=True)
        # self.monitor_thread.start()
        print("Indy connector has been initialised.")
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    def loop_mycobot_monitor(self):
        while True:
            try:
                if self.is_connected:
                    self.__di = self.mc.get_digital_input(1)
                    self.__q = self.mc.get_angles()
                    err = self.mc.get_error_information()
                    self.error_pub.publish(Int32(data=int(err)))
                time.sleep(1/self.pub_rate)
            except Exception as e:
                self.get_logger().error("Error in mycobot monitor loop")
                self.get_logger().error(str(e))

    def __enter_timecheck(self, fn_id):
        if self.save_log:
            self.callback_time_dict[fn_id] = time.time()

    def __exit_timecheck(self, fn_id):
        if self.save_log:
            self.callback_log.append([fn_id, self.callback_time_dict[fn_id], time.time()])


    def connect(self):
        self.mc = MyCobot("/dev/ttyAMA0", 1000000)
        self.is_connected = True
        init_pose = [[0, 0, 0, 0, 0, 0], self.speed]
        home_pose = [[0, 8, -127, 40, 0, 0], self.speed] # 0,8,-127,40,0,0
        
        self.mc.send_angles(*home_pose)
        time.sleep(1)
        # 홈 위치 이동하고 멈출때까지 대기
        while self.mc.is_moving():
            time.sleep(0.2)

        # 현재값이 받아지는지 확인.
        while True:
            self.res = self.mc.get_coords()
            if self.res:
                break
            time.sleep(0.1)

        self.record_coords = [self.res, self.speed, self.MODE]
        self.prev_coords = self.record_coords

        print("current coords: %s" % self.record_coords)
        print("tool reference: ", self.mc.get_tool_reference())
        print("world reference: ", self.mc.get_world_reference())
        print("reference frame: ", self.mc.get_reference_frame())
        print("end type: ", self.mc.get_end_type())

    def disconnect(self):
        self.is_connected = False
        self.stop_motion()
        self.get_logger().warning("DISCONNECT TO ROBOT")
        time.sleep(0.3)

    def get_mycobot_error_msg(self):
        err = self.mc.get_error_information()
        if err == 0:
            return "Normal"
        if err > 0 and err < 7 :
            return f"{err}joint exceeds the limit"
        if err > 15 and err < 20 :
            return "collision protection"
        if err == 32:
            return "Kinematics inverse solution has no solution"
        if err > 32 and err <35:
            return "Linear motion has no adjacent solution"
        return None


    # def joint_position_callback(self, msg):
    #     with BlockWrapper(partial(self.__enter_timecheck, 1), #코드 실행 시간을 측정하고 로깅을 수행
    #                       partial(self.__exit_timecheck, 1)):
    #         self.stop_motion()
    #         #관절 움직임 목록 추출
    #         joint_motion_list = [p.positions for p in msg.points]
    #
    #         #--------------reduce the trajectory points(경로 최적화)----------
    #         new_array = [joint_motion_list[0]]
    #         percentage = 50 # percentage of reduced point
    #         number = int(len(joint_motion_list) * (1 - (percentage/100))) - 2
    #         if number > 0:
    #             distance = int((len(joint_motion_list) - 2) / (number + 1))
    #             for i in range(1, number + 1):
    #                 new_array.append(joint_motion_list[i * distance])
    #         new_array.append(joint_motion_list[-1])
    #         #---------------------------------------------------
    #         self.start_joint_motion() ## 현재 선언되어있지않음
    #         for joint_rads in new_array: # 관전 목록에 있는 각 관절 위치로 로봇을 움직임
    #             joint_rads = list(map(float, joint_rads))
    #             self.move_joint(joint_rads) ## 현재 선언되어있지 않음
    #             self.get_logger().info('robot is moving...')
    #             while np.sum(np.abs(deg2rad(self.mc.get_angles()) - joint_rads)) > 1e-2:
    #                 time.sleep(0.1)


    def tele_move_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 2),
                          partial(self.__exit_timecheck, 2)):

            # 초기 텔레오퍼레이션 모드 대기 중인 경우(True)
            if self.wait_init_tele_mode:
                # 로봇의 오류 메시지를 확인
                error = self.get_mycobot_error_msg()
                if error is not None :
                    # 오류가 발생한 경우 경고 메시지를 출력
                    self.get_logger().warning(error + 'WHILE TELE-MOVING')
                else:
                    # 오류가 없으면 초기 텔레오퍼레이션 모드를 종료하고 텔레오퍼레이션 모드를 시작합니다.
                    self.wait_init_tele_mode = False
                    self.start_teleop()
                    self.get_logger().warning(f"Turn ON Tele Mode DONE For {self.robot_ip}")
                return

        self.update_teleop_target(msg.data)

    def start_teleop(self):
        task_cmd = self.mc.get_coords()  # mm, degs
        self.Tre_ref = cmd2T(task_cmd)
        self.Tre_ref[:3,3] /= 1000.0  # to m scale
        self.Trt_ref = np.matmul(self.Tre_ref, self.Tet)
        self.Tle = SE3(np.identity(3), self.Tre_ref)  # 엔드이펙터 로컬 좌표계 (로봇 방향 + 엔드이펙터 위치)
        self.Tlt = SE3(np.identity(3), self.Trt_ref)  # 툴 로컬 좌표계 (로봇 방향 + 툴 위치)
        self.Tet = np.matmul(np.linalg.inv(self.Tle), self.Tlt)
        self.Tte = np.linalg.inv(self.Tet)
        self.Telre = np.matmul(np.linalg.inv(self.Tle), self.Tre_ref)  # 엔드이펙터 로컬 좌표계 <- 원래 엔드이펙터 좌표계 변환 행렬
        self.tele_mode_on = True

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

            # 툴 로컬 좌표계 변환 행렬을 엔드이펙터 로컬 좌표계 변환 행렬로 전환 (기준 엔드이펙터 로컬 좌표계 기준, 새 엔드이펙터 좌표계로의 변환 행렬)
            # T(e<-e2) = T(e<-t)*T(t<-t2)*T(t2<-e2) = Tet*dTt*Tte
            dTe = np.matmul(self.Tet, np.matmul(dTt, self.Tte))

            # 엔드이펙터 로컬 좌표계 변환 행렬이 적용 된 새 엔드이펙터 로컬 좌표계 획득
            Tle_new = np.matmul(self.Tle, dTe)

            # 새 엔드이펙터 로컬 좌표계로부터 원래 엔드이펙터의 새 좌표계 획득
            Tre_new = np.matmul(Tle_new, self.Telre)

            # 로봇 베이스 기준 엔드이펙터 위치 및 방향 추출
            xyz_re = Tre_new[:3,3] * 1000  # 엔드이펙터의 새 위치, mm 단위로 변환
            rpy_re = rad2deg(Rot2rpy(Tre_new[:3,:3]))  # 엔드이펙터의 새 RPY각, deg 단위로 변환

            # 업데이트된 위치와 회전 정보(xyz_re 및 rvec_re)를 사용하여 로봇의 텔레오퍼레이션 궤적을 업데이트합니다.
            # send_coords는 베이스 기준 엔드이펙터의 mm 단위 위치와 deg 단위 rpy 각도를 입력 받습니다.
            self.mc.send_coords(np.concatenate([xyz_re, rpy_re]).tolist(), self.speed, self.MODE)  # 이것은 로봇을 새로운 위치와 회전으로 이동시키는 데 사용됩니다.

    # def start_joint_motion(self):

    # def move_joint(self, joint_rads):

    def stop_motion(self):
        self.get_logger().info("STOP CALLED")
        self.joint_ref = None
        self.mc.stop()
        self.tele_mode_on = False


    def tele_mode_callback(self, msg):
        with BlockWrapper(partial(self.__enter_timecheck, 2), #코드 실행 시간을 측정하고 로깅을 수행
                          partial(self.__exit_timecheck, 2)):
            if msg.data is True:
                self.stop_motion()
                # self.indy.joint_move_to(rad2deg(self.home_pose).tolist())
                time.sleep(0.1)
                self.wait_init_tele_mode = True
            elif msg.data is False:
                self.stop_motion()

    def finger_callback(self,msg):
        with BlockWrapper(partial(self.__enter_timecheck, 3),
                          partial(self.__exit_timecheck, 3)):
            self.enable_tele = msg.data
            # di_list = self.__di.copy()
            # if di_list[self.gripper_di] != self.gripper_state:
            #     self.gripper_state = di_list[self.gripper_di]
            #     if self.gripper_state:
            #         self.mc.set_gripper_value(0, 50) #Gripper Close
            #         self.get_logger().info('Gripper close')
            #     else:
            #         self.mc.set_gripper_value(200, 50) #Gripper Open
            #         self.get_logger().info('Gripper open')

    def reset_callback(self,msg):
        self.mc.clear_error_information()

    def joint_state_publisher(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = deg2rad(self.__q).tolist()
        # joint_state_msg.velocity = self.indy.get_joint_vel() TODO
        # joint_state_msg.effort = get_control_torque() TODO
        self.joint_state_feedback.positions = self.joint_state_msg.position
        self.joint_state_pub.publish(self.joint_state_msg)

    def timer_callback(self):
        with BlockWrapper(partial(self.__enter_timecheck, 5),
                          partial(self.__exit_timecheck, 5)):
            coords_changed = False
            gripper_value = None
            self.joint_state_publisher()
            di_list = self.__di.copy()
            if di_list[self.gripper_di] != self.gripper_state:
                self.gripper_state = di_list[self.gripper_di]
                if self.gripper_state:
                    self.mc.set_gripper_value(0, 50) #Gripper Close
                    self.get_logger().info('Gripper close')
                else:
                    self.mc.set_gripper_value(200, 50) #Gripper Open
                    self.get_logger().info('Gripper open')

    # def goal_callback(self, goal_request):
    #     with BlockWrapper(partial(self.__enter_timecheck, 6),
    #                       partial(self.__exit_timecheck, 6)):
    #         # Accepts or rejects a client request to begin an action
    #         self.get_logger().info('Received goal request :)')
    #         return GoalResponse.ACCEPT
    #
    # def cancel_callback(self, goal_handle):
    #     with BlockWrapper(partial(self.__enter_timecheck, 7),
    #                       partial(self.__exit_timecheck, 7)):
    #         # Accepts or rejects a client request to cancel an action
    #         self.get_logger().info('Received cancel request :(')
    #         return CancelResponse.ACCEPT

    # async def execute_callback(self, goal_handle):



    # def sigma_callback(self, msg):
    #     res_copy = self.res.copy()
    #     res_copy[1] += (msg.pose.position.x * 1000) # *1000 => mm (sigma ~ -100mm to 100mm)
    #     res_copy[0] -= (msg.pose.position.y * 1000)
    #     res_copy[2] += (msg.pose.position.z * 1000)
    #
    #     roll, pitch, yaw = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    #     #print("roll: ", roll, " pitch: ", pitch, " yaw: ", yaw)
    #     res_copy[4] -= (roll / math.pi *180) # deg
    #     res_copy[3] += (pitch / math.pi *180) # deg
    #     res_copy[5] += ((yaw - (math.pi/2)) / math.pi *180) # offset (math.pi/2)
    #
    #     # Clamp the position within the limits
    #     res_copy[0] = min(max(res_copy[0], -self.MAX_LEN), self.MAX_LEN)
    #     res_copy[1] = min(max(res_copy[1], -self.MAX_LEN), self.MAX_LEN)
    #     res_copy[2] = min(max(res_copy[2], -self.MAX_LEN), self.MAX_LEN)
    #     res_copy[3] = min(max(res_copy[3], -self.MAX_ANGLE), self.MAX_ANGLE)
    #     res_copy[4] = min(max(res_copy[4], -self.MAX_ANGLE), self.MAX_ANGLE)
    #     res_copy[5] = min(max(res_copy[5], -self.MAX_ANGLE), self.MAX_ANGLE)
    #
    #
    #     # Update the record_coords variable with the new values
    #     self.record_coords[0] = res_copy
    #
    # def sigma_gripper_callback(self, msg):
    #     self.gripper = msg.data



def main(args=None):
    rclpy.init(args=args)

    with Sigma_MyCobot_Comm() as control_slider:
        executor = MultiThreadedExecutor()
        rclpy.spin(control_slider, excutor=executor) # ROS 2 노드(Node)를 실행하고, 노드 내에 등록된 콜백 함수들을 실행합니다.
        
    rclpy.shutdown()


if __name__ == "__main__":
    main()
