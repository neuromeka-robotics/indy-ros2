#!/usr/bin/python3
#-*- coding: utf-8 -*-
'''
    Neuromeka V-SCRC
    Nguyen Pham
'''

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

class Controller:
    AXES = {
        "L_JOY_LeftRight": 0,
        "L_JOY_UpDown": 1,
        "LT_Trigg": 2,
        "R_JOY_LeftRight": 3,
        "R_JOY_UpDown": 4,
        "RT_Trigg": 5,
        "D_PAD_LeftRight": 6,
        "D_PAD_UpDown": 7
    }
    BUTTONS = {
        "A": 0,
        "B": 1,
        "X": 2,
        "Y": 3,
        "LB": 4,
        "RB": 5,
        "BACK": 6,
        "START": 7,
        "XBOX": 8,
        "L_JOY": 9,
        "R_JOY": 10
    }


class IndyXboxConnection(Node):
    PUBLISH_RATE = 20 # Hz
    INIT_POS = [0.0]*6
    WORKING_RADIUS = 0.2

    def __init__(self):
        super().__init__('indy_xbox')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        self.declare_parameter('joy_prefix', "")
        self.declare_parameter('robot0', "robot0")
        self.declare_parameter('robot1', "robot1")
        joy_prefix = self.get_parameter('joy_prefix').get_parameter_value().string_value
        robot0_name = self.get_parameter('robot0').get_parameter_value().string_value + "_"
        robot1_name = self.get_parameter('robot1').get_parameter_value().string_value + "_"

        self.joy_stick_sub = self.create_subscription(
            Joy,
            joy_prefix + '/joy',
            self.joy_stick_callback,
            qos_profile
            )
        self.joy_stick_sub

        # Initialize topics
        self.timer = self.create_timer(1/self.PUBLISH_RATE, self.timer_callback)

        self.tele0_mode_pub = self.create_publisher(Bool, robot0_name + 'tele_mode', qos_profile)
        self.tele0_move_pub = self.create_publisher(Float32MultiArray, robot0_name + 'tele_move', qos_profile)

        self.tele1_mode_pub = self.create_publisher(Bool, robot1_name + 'tele_mode', qos_profile)
        self.tele1_move_pub = self.create_publisher(Float32MultiArray, robot1_name + 'tele_move', qos_profile)

        self.tele_mode_msg = Bool()
        self.tele_move_msg = Float32MultiArray()
        
        # Initialize variable
        self.switch_robot = False

        self.tele_mode = False
        self.tele_config = False

        # self.working_radius = 0.2  # m
        self.axes = Controller.AXES
        self.buttons = Controller.BUTTONS

        self.target_pos = self.INIT_POS.copy()
        self.target_offset = self.INIT_POS.copy()

        self.robot_variables = {
            "robot0": {
                "tele_mode": self.tele_mode,
                "target_pos": self.target_pos,
                "target_offset": self.target_offset,
                "tele_mode_pub": self.tele0_mode_pub,
                "tele_move_pub": self.tele0_move_pub,
            },
            "robot1": {
                "tele_mode": self.tele_mode,
                "target_pos": self.target_pos,
                "target_offset": self.target_offset,
                "tele_mode_pub": self.tele1_mode_pub,
                "tele_move_pub": self.tele1_move_pub,
            },
        }

    def switch_robot_func(self, robot_variables, switch_robot):
        current_robot = "robot0" if not switch_robot else "robot1"
        other_robot = "robot1" if not switch_robot else "robot0"
        
        # keep current pose when back to this robot later
        robot_variables[current_robot]["tele_mode"] = self.tele_mode
        robot_variables[current_robot]["target_pos"] = self.INIT_POS.copy()
        robot_variables[current_robot]["target_offset"] = self.target_pos

        self.tele_mode = robot_variables[other_robot]["tele_mode"]
        self.target_pos = robot_variables[other_robot]["target_pos"]
        self.target_offset = robot_variables[other_robot]["target_offset"]

        self.get_logger().warning(f"SWITCH TO {other_robot.upper()}")


    '''
    Xbox subscribe
    ''' 
    def joy_stick_callback(self, msg):

        # Usage example:
        if msg.buttons[Controller.BUTTONS["LB"]] and not self.switch_robot:
            self.switch_robot_func(self.robot_variables, self.switch_robot)
            self.switch_robot = True

        elif msg.buttons[Controller.BUTTONS["RB"]] and self.switch_robot:
            self.switch_robot_func(self.robot_variables, self.switch_robot)
            self.switch_robot = False

        if msg.buttons[Controller.BUTTONS["A"]] and not self.tele_mode:
            self.tele_mode = True
            self.tele_mode_msg.data = True
            self.target_pos = self.INIT_POS.copy()
            self.target_offset = self.INIT_POS.copy()

            current_robot = "robot0" if not self.switch_robot else "robot1"
            self.robot_variables[current_robot]["tele_mode_pub"].publish(self.tele_mode_msg)
            self.get_logger().warning(f"Turn ON Tele Mode FOR {current_robot.upper()}")

        elif msg.buttons[Controller.BUTTONS["B"]] and self.tele_mode:
            self.tele_mode = False
            self.tele_mode_msg.data = False

            current_robot = "robot0" if not self.switch_robot else "robot1"
            self.robot_variables[current_robot]["tele_mode_pub"].publish(self.tele_mode_msg)
            self.get_logger().warning(f"Turn OFF Tele Mode FOR {current_robot.upper()}")


        if self.tele_mode:
            '''
            In tele mode:
                Use L_JOY_UpDown, L_JOY_LeftRight to move follows x, y
                Use R_JOY_LeftRight, R_JOY_UpDown to move follows roll, pitch
                Press Start to enter config mode
            In config mode:
                Use L_JOY_UpDown, L_JOY_LeftRight to move follows x, y
                Use R_JOY_LeftRight, R_JOY_UpDown to move follows roll, pitch
                Use D_PAD_UpDown to move z
            '''
            if not self.tele_config:
                self.target_pos[0] = (msg.axes[self.axes["L_JOY_UpDown"]] * self.WORKING_RADIUS) + self.target_offset[0]
                self.target_pos[1] = (msg.axes[self.axes["L_JOY_LeftRight"]] * self.WORKING_RADIUS) + self.target_offset[1]
                # self.target_pos[2] = (msg.axes[self.axes["D_PAD_UpDown"]] * self.WORKING_RADIUS) + self.target_offset[2]
                self.target_pos[3] = (msg.axes[self.axes["R_JOY_LeftRight"]] * (-math.pi / 3)) + self.target_offset[3]
                self.target_pos[4] = (msg.axes[self.axes["R_JOY_UpDown"]] * (math.pi / 3)) + self.target_offset[4]

                if msg.buttons[self.buttons["START"]]:
                    self.get_logger().info("Enter Tele Config")
                    self.tele_config = True

            elif self.tele_config:
                self.target_offset[0] += (msg.axes[self.axes["L_JOY_UpDown"]] * (self.WORKING_RADIUS / 50))
                self.target_offset[1] += (msg.axes[self.axes["L_JOY_LeftRight"]] * (self.WORKING_RADIUS / 50))
                self.target_offset[2] += (msg.axes[self.axes["D_PAD_UpDown"]] * (self.WORKING_RADIUS / 50))
                self.target_offset[3] += (msg.axes[self.axes["R_JOY_LeftRight"]] * (-math.pi / 150))
                self.target_offset[4] += (msg.axes[self.axes["R_JOY_UpDown"]] * (math.pi / 150))

                # to show the moving
                self.target_pos[0] = self.target_offset[0]
                self.target_pos[1] = self.target_offset[1]
                self.target_pos[2] = self.target_offset[2]
                self.target_pos[3] = self.target_offset[3]
                self.target_pos[4] = self.target_offset[4]

                if msg.buttons[self.buttons["BACK"]]:
                    self.get_logger().info("Exit Tele Config")
                    self.tele_config = False


    '''
    Xbox publish
    '''
    def timer_callback(self):
        self.tele_move_msg.data = self.target_pos
        current_robot = "robot0" if not self.switch_robot else "robot1"
        self.robot_variables[current_robot]["tele_move_pub"].publish(self.tele_move_msg)

def main(args=None):
    rclpy.init(args=args)
    gamepad_service_client = IndyXboxConnection()
    rclpy.spin(gamepad_service_client)
    gamepad_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
