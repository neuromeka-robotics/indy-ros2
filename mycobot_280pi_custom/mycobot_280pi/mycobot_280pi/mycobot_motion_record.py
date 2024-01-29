#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Simple Motion Record for Mycobot 280pi
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


main_msg = """
Reading cmd from the keyboard
---------------------------
Start Record: r (this cmd will overwrite current record)
Play Record: p (play latest record)
---------------------------
Press q and CTRL-C to exit
"""

record_msg = """
Reading cmd from the keyboard
---------------------------
Recording ...
Press space to stop.
---------------------------
Press q and CTRL-C to exit
"""

play_msg = """
Reading cmd from the keyboard
---------------------------
Playing ...
Press space to stop.
---------------------------
Press q and CTRL-C to exit
"""

STATE_MAIN = 0
STATE_RECORD = 1
STATE_PLAY = 2

JOINT_MOTION = 0
GRIPPER_MOTION = 1

FREQUENCY = 20 # Hz

class MyCobot_Motion_Record(Node):

    def __init__(self):
        super().__init__("mycobot_motion_record")
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.declare_parameter('robot', "robot0")
        self.robot_name = self.get_parameter('robot').get_parameter_value().string_value + "_"

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile
        )
        self.joint_state_sub

        self.gripper_sub = self.create_subscription(
            UInt16,
            "/" + self.robot_name + "control_gripper_range",
            self.control_gripper_range_callback,
            qos_profile
        )
        self.gripper_sub

        self.timer_ = self.create_timer(1 / FREQUENCY, self.timer_callback)
        # self.coords_pub = self.create_publisher(Float32MultiArray, self.robot_name + 'control_coords', qos_profile)
        self.radians_pub = self.create_publisher(Float32MultiArray, self.robot_name + 'control_radians', qos_profile)
        self.gripper_pub = self.create_publisher(UInt16, self.robot_name + 'control_gripper_range', qos_profile)

        self.print_request = True
        self.state_machine = STATE_MAIN
        self.play_pos = 0
        
        self.prev_gripper_range = None
        self.motion_list = []
        #self.joint_state_feedback = None
        self.shutdown_requested = False

        self.key_thread = threading.Thread(target=self.key_input_thread)
        self.key_thread.daemon = True
        self.key_thread.start()

    def key_input_thread(self):
        while not self.shutdown_requested:
            if self.print_request:
                if self.state_machine == STATE_MAIN:
                    print(main_msg)
                elif self.state_machine == STATE_RECORD:
                    print(record_msg)
                elif self.state_machine == STATE_PLAY:
                    print(play_msg)
                self.print_request = False
                self.play_pos = 0

            key = self.getKey(self.saveTerminalSettings(), 0.5)
            if key == 'q':
                self.get_logger().info("Exiting ROS Python script...")
                self.shutdown_requested = True
                break
            elif key == 'r':
                self.state_machine = STATE_RECORD
                self.motion_list = []
                self.print_request = True
            elif key == 'p':
                self.state_machine = STATE_PLAY
                self.print_request = True
            elif key == ' ':
                self.state_machine = STATE_MAIN
                self.print_request = True
            time.sleep(0.02)

    def joint_state_callback(self, msg: JointState):
        # self.joint_state_feedback = msg.position
        robot_name = msg.name[0].split("_")[0] + "_"
        if self.state_machine == STATE_RECORD and self.robot_name == robot_name:
            self.motion_list.append([JOINT_MOTION, msg.position[0:6]])

    def control_gripper_range_callback(self, msg):
        # self.gripper_range = msg.data
        if self.state_machine == STATE_RECORD:
            self.motion_list.append([GRIPPER_MOTION, msg.data])

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

        if self.state_machine == STATE_PLAY:
            if self.play_pos < len(self.motion_list):
                cmd, data = self.motion_list[self.play_pos]
                # print("cmd: ", cmd, " ", "data: ", self.prev_gripper_range)
                if cmd == JOINT_MOTION and data is not None:
                    msg = Float32MultiArray()
                    float_radians = [float(value) for value in data]
                    msg.data = float_radians
                    self.radians_pub.publish(msg)
                elif cmd == GRIPPER_MOTION and data is not None: # and data != self.prev_gripper_range:
                    # print("Gripper")
                    msg_gripper = UInt16()
                    msg_gripper.data = int(data)
                    self.gripper_pub.publish(msg_gripper)
                    # self.prev_gripper_range = data
                self.play_pos += 1
            else:
                self.state_machine = STATE_MAIN
                self.print_request = True

def main(args=None):
    rclpy.init(args=args)
    mycobot_motion_record = MyCobot_Motion_Record()
    try:
        rclpy.spin(mycobot_motion_record)
    except KeyboardInterrupt:
        pass
    mycobot_motion_record.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
