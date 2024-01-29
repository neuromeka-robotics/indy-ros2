#!/usr/bin/python3
#-*- coding: utf-8 -*-

"""
    Moveit and Mycobot Interface
    Please refer to the readme file.
    Author: Nguyen Pham
"""

import rclpy
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory


class Moveit_MyCobot_Comm(Node):

    HOME_POS_COORDS = [149.8, -63.7, 159.1, -170.68, -0.47, -89.81]
    MAX_LEN = 280 # mm
    MAX_ANGLE = 270 # deg

    def __init__(self):
        super().__init__("moveit_mycobot_comm")
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.declare_parameter('robot', "robot0")
        robot = self.get_parameter('robot').get_parameter_value().string_value + "_"

        self.jtc_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/" + robot + 'joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile
        )
        self.joint_state_sub

        self.coords_pub = self.create_publisher(Float32MultiArray, robot + 'control_coords', qos_profile)
        self.radians_pub = self.create_publisher(Float32MultiArray, robot + 'control_radians', qos_profile)

        self.joint_state_list = []
        self.joint_state_feedback = None

    def joint_state_callback(self, msg: JointState):
        self.joint_state_feedback = msg.position

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        print('FollowJointTrajectory callback...')
        feedback_msg = FollowJointTrajectory.Feedback()
        # last_time = self.get_clock().now()

        # download planned path from ros moveit
        self.joint_state_list = []
        if self.goal.trajectory.points:
            self.joint_state_list = [p.positions for p in self.goal.trajectory.points]
            # print(self.joint_state_list)
            if self.joint_state_list:
                for j_pos in self.joint_state_list:
                    msg = Float32MultiArray()
                    float_radians = [float(value) for value in j_pos]
                    msg.data = float_radians
                    self.radians_pub.publish(msg)

                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        print("Trajectory canceled")
                        return FollowJointTrajectory.Result()

                    feedback_msg.desired.positions = j_pos
                    feedback_msg.actual.positions = self.joint_state_feedback
                    goal_handle.publish_feedback(feedback_msg)

                    time.sleep(0.05)

                self.joint_state_list = []

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result
    
def main(args=None):
    rclpy.init(args=args)
    moveit_mycobot = Moveit_MyCobot_Comm()
    
    rclpy.spin(moveit_mycobot)
    
    moveit_mycobot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
