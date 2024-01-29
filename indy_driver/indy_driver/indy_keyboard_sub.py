#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Simple Motion Record for Indy
    Please refer to the readme file.
    Author: Nguyen Pham from Neuromeka
"""

from __future__ import print_function

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

import rclpy
import math
import time
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32MultiArray, UInt16, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from ament_index_python.packages import get_package_share_directory

import sys
import os
DRIVER_DIR = get_package_share_directory('indy_driver')
sys.path.append(os.path.join(DRIVER_DIR, "nrmk_utils"))
from file_io import *

def prRed(skk): print("\033[91m {}\033[00m" .format(skk))
def prGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def prYellow(skk): print("\033[93m {}\033[00m" .format(skk))
def prLightPurple(skk): print("\033[94m {}\033[00m" .format(skk))
def prPurple(skk): print("\033[95m {}\033[00m" .format(skk))


JOINT_MOTION = 0
GRIPPER_MOTION = 1

qos_profile = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE
)


class Robot(Node):
    def __init__(self, robot_name):
        super().__init__("RobotNode_" + robot_name)
        self.robot_name = robot_name
        self.tele_mode = False
        self.pose_pub = self.create_publisher(JointTrajectory, self.robot_name + '_joint_trajectory', qos_profile)
    # def motion_list_pub(self):
    #     pub_list = []
    #     for cmd, data in self.motion_list:
    #         if cmd == JOINT_MOTION and data is not None:
    #             p = JointTrajectoryPoint()
    #             p.positions = list(data)
    #             # p.velocities = [0]
    #             # p.accelerations = [0]
    #             pub_list.append(p)

    #     traj = JointTrajectory()
    #     traj.points = pub_list
    #     self.pose_pub.publish(traj)

class Sigma(Node):
    def __init__(self, sigma_name, robot1, robot2):
        super().__init__("SigmaNode_" + sigma_name)
        self.sigma_name = sigma_name
        self.switch_setting = False
        self.robot1 = robot1
        self.robot2 = robot2
        self.sigma_tele_pub = self.create_publisher(Bool, self.sigma_name + '_tele_mode', qos_profile)
        self.sigma_sw_pub = self.create_publisher(Bool, self.sigma_name + '_switch_robot', qos_profile)
        self.sigma_scale_pub = self.create_publisher(Float32MultiArray, self.sigma_name + '_scale', qos_profile)
        
    def get_active_robot(self):
        if self.switch_setting:
            return self.robot2
        else:
            return self.robot1

    def set_scale(self, pos, rot):
        robot = self.get_active_robot()
        resume_tele_mode = False
        if robot.tele_mode:
            self.set_tele_mode(status=False)
            resume_tele_mode = True
            time.sleep(0.5)
        self.sigma_scale_pub.publish(Float32MultiArray(data=[pos, rot]))
        if resume_tele_mode:
            time.sleep(0.5)
            self.set_tele_mode(status=True)
        
    def set_tele_mode(self, status = None):
        robot = self.get_active_robot()
        robot.tele_mode = status if status is not None else not robot.tele_mode
        msg = Bool()
        msg.data = robot.tele_mode
        return self.sigma_tele_pub.publish(msg)

    def switch_robot(self):
        self.switch_setting = not self.switch_setting
        msg = Bool()
        msg.data = self.switch_setting
        return self.sigma_sw_pub.publish(msg)


class KeyboardInterface(Node):

    STATE_MAIN = 0
    STATE_SUB = 1
    STATE_RECORD = 2
    STATE_PLAY = 3
    STATE_MANUAL_INPUT = 4
    STATE_MANUAL_MOVE = 5
    
    FREQUENCY = 20 # Hz
  
    def __init__(self):
        super().__init__("KeyboardInterface")
        # qos_profile = QoSProfile(
        #     depth=10,
        #     reliability=QoSReliabilityPolicy.RELIABLE
        # )

        self.declare_parameter('sigma0_name', "sigma0")
        self.declare_parameter('sigma1_name', "sigma1")

        self.sigma0_name = self.get_parameter('sigma0_name').get_parameter_value().string_value
        self.sigma1_name = self.get_parameter('sigma1_name').get_parameter_value().string_value

        self.declare_parameter('robot0', "robot0")
        self.declare_parameter('robot1', "robot1")
        self.declare_parameter('robot2', "robot2")
        self.declare_parameter('robot3', "robot3")
        self.robot0_name = self.get_parameter('robot0').get_parameter_value().string_value
        self.robot1_name = self.get_parameter('robot1').get_parameter_value().string_value
        self.robot2_name = self.get_parameter('robot2').get_parameter_value().string_value
        self.robot3_name = self.get_parameter('robot3').get_parameter_value().string_value

        self.robot0 = Robot(self.robot0_name)
        self.robot1 = Robot(self.robot1_name)
        self.robot2 = Robot(self.robot2_name)
        self.robot3 = Robot(self.robot3_name)
        
        self.manual_target_pos =[0]*6
        self.manual_pos_list = ['X','Y','Z','U','V','W']
        
        
        self.declare_parameter('mode_configs', 'mode_configs.yaml')
        mode_configs = self.get_parameter('mode_configs').get_parameter_value().string_value
        config_dicts = load_yaml(os.path.join(DRIVER_DIR, "config", mode_configs))
        self.mode_scales = []
        for i in range(0, 10):
            _mode_name = f'mode{i}'
            if _mode_name in config_dicts:
                _pos = config_dicts[_mode_name]['pos']
                _rot = config_dicts[_mode_name]['rot']
            else:
                _pos, _rot = 1.0, 1.0
            self.mode_scales.append((_pos, _rot))

        self.sigma0 = Sigma(self.sigma0_name, self.robot0, self.robot1)
        self.sigma1 = Sigma(self.sigma1_name, self.robot2, self.robot3)

        self.sigma_dict = {
            self.sigma0_name: self.sigma0,
            self.sigma1_name: self.sigma1
        }
        self.current_sigma = self.sigma0_name

        self.save_log_status = False
        self.save_log_pub = self.create_publisher(Bool, "save_log_status", qos_profile)
        self.record_status = False
        # self.record_pub = self.create_publisher(Bool, "record_status", qos_profile)
        self.robot0_record_pub = self.create_publisher(Bool, 'robot0' + '_record_status', qos_profile)
        self.robot1_record_pub = self.create_publisher(Bool, 'robot1' + '_record_status', qos_profile)
        self.robot2_record_pub = self.create_publisher(Bool, 'robot2' + '_record_status', qos_profile)
        self.robot3_record_pub = self.create_publisher(Bool, 'robot3' + '_record_status', qos_profile)
        self.play_status = False
        # self.play_pub = self.create_publisher(Bool, "play_status", qos_profile)
        self.robot0_play_pub = self.create_publisher(Bool, 'robot0' + '_play_status', qos_profile)
        self.robot1_play_pub = self.create_publisher(Bool, 'robot1' + '_play_status', qos_profile)
        self.robot2_play_pub = self.create_publisher(Bool, 'robot2' + '_play_status', qos_profile)
        self.robot3_play_pub = self.create_publisher(Bool, 'robot3' + '_play_status', qos_profile)
            
        self.record_pubs = {
        self.robot0_name: {"record_pub": self.robot0_record_pub},
        self.robot1_name: {"record_pub": self.robot1_record_pub},
        self.robot2_name: {"record_pub": self.robot2_record_pub},
        self.robot3_name: {"record_pub": self.robot3_record_pub},
        }
        self.play_pubs = {
        self.robot0_name: {"play_pub": self.robot0_play_pub},
        self.robot1_name: {"play_pub": self.robot1_play_pub},
        self.robot2_name: {"play_pub": self.robot2_play_pub},
        self.robot3_name: {"play_pub": self.robot3_play_pub},
        }

        self.robot0_manual_pub = self.create_publisher(Float32MultiArray, 'robot0' + '_manual_motion', qos_profile)
        self.robot1_manual_pub = self.create_publisher(Float32MultiArray, 'robot1' + '_manual_motion', qos_profile)
        self.robot2_manual_pub = self.create_publisher(Float32MultiArray, 'robot2' + '_manual_motion', qos_profile)
        self.robot3_manual_pub = self.create_publisher(Float32MultiArray, 'robot3' + '_manual_motion', qos_profile)

        self.manual_pubs = {
            self.robot0_name: {"manual_motion": self.robot0_manual_pub},
            self.robot1_name: {"manual_motion": self.robot1_manual_pub},
            self.robot2_name: {"manual_motion": self.robot2_manual_pub},
            self.robot3_name: {"manual_motion": self.robot3_manual_pub}
        }

        self.robot0_reset_pub = self.create_publisher(Bool, 'robot0' + '_reset', qos_profile)
        self.robot1_reset_pub = self.create_publisher(Bool, 'robot1' + '_reset', qos_profile)
        self.robot2_reset_pub = self.create_publisher(Bool, 'robot2' + '_reset', qos_profile)
        self.robot3_reset_pub = self.create_publisher(Bool, 'robot3' + '_reset', qos_profile)

        self.reset_pubs = {
            self.robot0_name: {"reset": self.robot0_reset_pub},
            self.robot1_name: {"reset": self.robot1_reset_pub},
            self.robot2_name: {"reset": self.robot2_reset_pub},
            self.robot3_name: {"reset": self.robot3_reset_pub}
        }

        self.robot0_home_pub = self.create_publisher(Bool, 'robot0' + '_home', qos_profile)
        self.robot1_home_pub = self.create_publisher(Bool, 'robot1' + '_home', qos_profile)
        self.robot2_home_pub = self.create_publisher(Bool, 'robot2' + '_home', qos_profile)
        self.robot3_home_pub = self.create_publisher(Bool, 'robot3' + '_home', qos_profile)

        self.home_pubs = {
            self.robot0_name: {"home": self.robot0_home_pub},
            self.robot1_name: {"home": self.robot1_home_pub},
            self.robot2_name: {"home": self.robot2_home_pub},
            self.robot3_name: {"home": self.robot3_home_pub}
        }

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile
        )
        
        # self.gripper_sub = self.create_subscription(
        #     UInt16,
        #     "/" + self.robot_name + "control_gripper_range",
        #     self.control_gripper_range_callback,
        #     qos_profile
        # )
        # self.gripper_sub
        
        self.timer_ = self.create_timer(1 / self.FREQUENCY, self.timer_callback)
        
        self.print_request = True
        self.state_machine = self.STATE_MAIN
        
        # self.prev_gripper_range = None
        self.shutdown_requested = False

        self.key_thread = threading.Thread(target=self.key_input_thread)
        self.key_thread.daemon = True
        self.key_thread.start()

    def go_home(self):
        msg = Bool()
        msg.data = True
        robot = self.get_active_sigma().get_active_robot()
        if robot.tele_mode:
            self.get_active_sigma().set_tele_mode(status=False)
            time.sleep(0.5)
        self.home_pubs[self.get_active_sigma().get_active_robot().robot_name]["home"].publish(msg)
        # time.sleep(3)
        # if resume_tele_mode:
        #     self.get_active_sigma().set_tele_mode(status=True)

    def reset_indy(self):
        msg = Bool()
        msg.data = True
        return self.reset_pubs[self.get_active_sigma().get_active_robot().robot_name]["reset"].publish(msg)

    def manual_joint_move_to(self):
        msg = Float32MultiArray()
        msg.data = list(map(float, self.manual_target_pos))
        return self.manual_pubs[self.get_active_sigma().get_active_robot().robot_name]["manual_motion"].publish(msg)
        
    def toggle_play_status(self):
        self.play_status = not self.play_status
        msg = Bool()
        msg.data = self.play_status
        return self.play_pubs[self.get_active_sigma().get_active_robot().robot_name]["play_pub"].publish(msg)

    def toggle_recording_status(self, condition):
        self.record_status = condition
        msg = Bool()
        msg.data = self.record_status
        return self.record_pubs[self.get_active_sigma().get_active_robot().robot_name]["record_pub"].publish(msg)
    
    def toggle_save_log_status(self):
        self.save_log_status = not self.save_log_status
        msg = Bool()
        msg.data = self.save_log_status
        return self.save_log_pub.publish(msg)

    def get_active_sigma(self):
        return self.sigma_dict[self.current_sigma]

    def tele_mode_toggle(self, status=None):
        self.get_active_sigma().set_tele_mode(status)


    def key_input_thread(self):
        while not self.shutdown_requested:
            if self.print_request:
                if self.state_machine == self.STATE_MAIN:
                    prGreen(f"""
                    ===========================
                    1. SETTINGS FOR SIGMA0
                    2. SETTINGS FOR SIGMA1
                    ---------------------------
                    Press Q and CTRL-C to exit
                    """)

                elif self.state_machine == self.STATE_SUB:
                    prYellow(f"""
                    ---------------------------
                    {self.get_active_sigma().sigma_name} | {self.get_active_sigma().get_active_robot().robot_name} | TELE {self.get_active_sigma().get_active_robot().tele_mode}
                    ---------------------------
                    P: Play Record (play latest record)
                    S : Switch Robot
                    M: Manual move
                    E: Reset Robot
                    ---------------------------
                    Logging : {self.save_log_status}
                    ---------------------------
                    Press SPACE to back.
                    Press Q and CTRL-C to exit
                    """)


                elif self.state_machine == self.STATE_RECORD:
                    prPurple(f"""
                    ---------------------------
                    {self.get_active_sigma().sigma_name} | {self.get_active_sigma().get_active_robot().robot_name} | TELE {self.get_active_sigma().get_active_robot().tele_mode}
                    ---------------------------
                    Recording ...
                    ---------------------------
                    Logging : {self.save_log_status}
                    ---------------------------
                    Press SPACE to stop.
                    Press Q and CTRL-C to exit
                    """)

                elif self.state_machine == self.STATE_PLAY:
                    prPurple(f"""
                    ---------------------------
                    {self.get_active_sigma().sigma_name} | {self.get_active_sigma().get_active_robot().robot_name} | TELE {self.get_active_sigma().get_active_robot().tele_mode}
                    ---------------------------
                    Playing ...
                    ---------------------------
                    Logging : {self.save_log_status}
                    ---------------------------
                    Press SPACE to stop.
                    Press Q and CTRL-C to exit
                    """)
                elif self.state_machine == self.STATE_MANUAL_INPUT:
                    prPurple(f"""
                    ---------------------------
                    {self.get_active_sigma().sigma_name} | {self.get_active_sigma().get_active_robot().robot_name} | TELE {self.get_active_sigma().get_active_robot().tele_mode}
                    ---------------------------
                    Please Input the joint pos
                    
                    ---------------------------
                    Logging : {self.save_log_status}
                    ---------------------------
                    Input non float value to exit
                    """)
                    
                    while(True):
                        try:
                            data = input("Please input data list x,y,z,u,v,w : ").split(',')
                            self.manual_target_pos = [float(value.strip()) for value in data]
                            if self.manual_target_pos is not None and len(self.manual_target_pos) == 6:
                                break
                            self.get_logger().info("This value is incorrect. Please enter 6 values X,Y,Z,U,V,W")
                        except:
                            self.get_logger().info("This value is incorrect. Please enter a value of type float")

                        
                                
                    prPurple(f"""
                    ---------------------------
                    {self.get_active_sigma().sigma_name} | {self.get_active_sigma().get_active_robot().robot_name} | TELE {self.get_active_sigma().get_active_robot().tele_mode}
                    ---------------------------
                    X : {self.manual_target_pos[0]}
                    Y : {self.manual_target_pos[1]}
                    Z : {self.manual_target_pos[2]}
                    U : {self.manual_target_pos[3]}
                    V : {self.manual_target_pos[4]}
                    W : {self.manual_target_pos[5]}
                    
                    Please check again before move
                    ---------------------------
                    Press ']' if you want to move
                    ---------------------------
                    Press SPACE to stop.
                    Press Q and CTRL-C to exit
                    """)
                    self.state_machine = self.STATE_MANUAL_MOVE

                self.print_request = False

            key = self.getKey(self.saveTerminalSettings(), 0.5)

            if key == 'q':
                self.get_logger().info("Exiting ROS Python script...")
                self.shutdown_requested = True
                break
            elif self.state_machine == self.STATE_MAIN:
                if key == '1':
                    self.state_machine = self.STATE_SUB
                    self.current_sigma = self.sigma0_name
                elif key == '2':
                    self.state_machine = self.STATE_SUB
                    self.current_sigma = self.sigma1_name
                elif key == ' ':
                    pass
            else:
                if self.state_machine != self.STATE_PLAY:
                    if key and key in "0123456789":
                        self.get_logger().info(f"set_scale: {self.mode_scales[int(key)]}")
                        self.get_active_sigma().set_scale(*self.mode_scales[int(key)])
                    #if key == 't':
                        #self.tele_mode_toggle()

                if self.state_machine == self.STATE_SUB:
                    #if key == 'r':
                        #self.state_machine = self.STATE_RECORD
                        #self.toggle_recording_status(True)
                    if key == 'p':
                        self.tele_mode_toggle(status=False)
                        time.sleep(0.1)
                        self.state_machine = self.STATE_PLAY
                    elif key == 's':
                        self.get_active_sigma().switch_robot()
                    #elif key == 'v':
                        #self.toggle_save_log_status()
                    elif key == 'm':
                        self.tele_mode_toggle(status=False)
                        self.state_machine = self.STATE_MANUAL_INPUT
                    elif key == 'e':
                        self.get_logger().info("Reset Indy")
                        self.reset_indy()
                    #elif key == 'h':
                        #self.get_logger().info("Indy go home")
                        #self.go_home()
                    elif key == ' ':
                        self.state_machine = self.STATE_MAIN
                        if self.record_status:
                            self.toggle_recording_status(False)

                elif self.state_machine == self.STATE_MANUAL_MOVE:
                    if key == ']':
                        self.manual_joint_move_to()
                        self.state_machine = self.STATE_SUB
                    elif key == ' ':
                        self.state_machine = self.STATE_SUB

                else:
                    if key == ' ':
                        self.state_machine = self.STATE_SUB
                        if self.record_status:
                            self.toggle_recording_status(False)

            if key != '':
                self.print_request = True
            
            time.sleep(0.02)
            
    def joint_state_callback(self, msg: JointState):
        robot_name = msg.name[0].split("_")[0]
        # if self.state_machine == self.STATE_RECORD and self.get_active_sigma().get_active_robot().robot_name == robot_name:
            # self.get_active_sigma().get_active_robot().motion_list.append([JOINT_MOTION, msg.position[:]])

    # def control_gripper_range_callback(self, msg):
    #     # self.gripper_range = msg.data
    #     if self.state_machine == STATE_RECORD:
    #         self.motion_list.append([GRIPPER_MOTION, msg.data])

    @staticmethod
    def saveTerminalSettings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    @staticmethod
    def getKey(settings, timeout):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            rlist, _, _ = select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def timer_callback(self):
        if self.shutdown_requested:
            rclpy.shutdown()

        if self.state_machine == self.STATE_PLAY:
            # self.get_active_sigma().get_active_robot().motion_list_pub()
            self.toggle_play_status()
            self.state_machine = self.STATE_SUB
            self.print_request = True

def main(args=None):
    rclpy.init(args=args)
    kb_interface = KeyboardInterface()
    try:
        rclpy.spin(kb_interface)
    except KeyboardInterrupt:
        pass
    kb_interface.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
