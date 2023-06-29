#!/usr/bin/python3
#-*- coding: utf-8 -*-
# import sys
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

import grpc
from gRPC import ros_interface_pb2 as IndyRosInterface
from gRPC import ros_interface_pb2_grpc as IndyRosInterface_grpc


def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads


class IndyROSConnector(Node):

    PUBLISH_RATE = 20 # Hz

    def __init__(self):
        super().__init__('indy_driver')
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )

        # Initialize joint control servers

        self.jtc_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            )

        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_trajectory_callback,
            qos_profile
            )
        self.joint_trajectory_sub  # prevent unused variable warning

        # Initialize topics
        self.timer = self.create_timer(1/self.PUBLISH_RATE, self.timer_callback)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        # Initialize parameters  with default values
        self.declare_parameter('indy_ip', "127.0.0.1")
        self.declare_parameter('indy_type', "indy7")
        self.indy_ip = None
        self.indy_type = None

        # Initialize variable
        self.vel = 3 # level 1 -> 3
        self.blend = 0.2 # rad 0 -> 0.4
        self.joint_state_list = []
        self.joint_state_feedback = JointTrajectoryPoint()
        self.execute = False
        self.previous_joint_trajectory_sub = None

        self.grpc_channel = None
        self.grpc_stub = None

        print("Indy connector has been initialised.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

    '''
    Connecting to Indy
    '''
    # Connect to Indy
    def connect(self):
        self.indy_ip = self.get_parameter('indy_ip').get_parameter_value().string_value
        self.indy_type = self.get_parameter('indy_type').get_parameter_value().string_value
        print("ROBOT IP: ", self.indy_ip)
        print("ROBOT TYPE: ", self.indy_type)

        self.grpc_channel = grpc.insecure_channel(self.indy_ip + ':50055')
        self.grpc_stub = IndyRosInterface_grpc.IndyServiceStub(self.grpc_channel)

        self.grpc_stub.SetJointVelocityLevel(IndyRosInterface.JointVelocityLevelReq(vel=self.vel))
        self.grpc_stub.SetJointBlending(IndyRosInterface.JointBlendingReq(blending_rad=self.blend))
        # self.grpc_stub.StartTeleMode(IndyRosInterface.Empty())
        time.sleep(0.3)

    # Disconnect to Indy
    def disconnect(self):
        print("DISCONNECT TO ROBOT")
        self.grpc_stub.StopRobot(IndyRosInterface.Empty())
        # self.grpc_stub.StopTeleMode(IndyRosInterface.Empty())
        time.sleep(0.3)
        self.grpc_channel.close()


    '''
    Indy subscribe
    ''' 
    def joint_trajectory_callback(self, msg):
        joint_state_list = []
        if msg.points:
            joint_state_list = [p.positions for p in msg.points]
        else:
            self.grpc_stub.StopRobot(IndyRosInterface.Empty())
        # print("joint state list: ", joint_state_list) #rad/s rad
        if self.previous_joint_trajectory_sub != joint_state_list[0]:
            self.grpc_stub.MoveJoint(IndyRosInterface.MoveJointReq(jpos=joint_state_list[0],
                                    base_type=IndyRosInterface.JOINT_BASE_TYPE_ABSOLUTE))
            # TODO: in evaluation, do not use TeleMode
            # self.grpc_stub.SetTeleTarget(IndyRosInterface.SetTeleTargetReq(target_pos=joint_state_list[0]))
            self.previous_joint_trajectory_sub = joint_state_list[0]
        

    '''
    Indy publish
    '''
    # Publish jointstates
    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.indy_type == 'indyrp2' or self.indy_type == 'indyrp2_v2':
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        else:
            joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        joint_state_msg.position = self.grpc_stub.GetJointPos(IndyRosInterface.Empty()).jpos
        # joint_state_msg.velocity = self.indy.get_joint_vel() TODO
        # joint_state_msg.effort = get_control_torque() TODO

        self.joint_state_feedback.positions = joint_state_msg.position
        self.joint_state_pub.publish(joint_state_msg)
        
    # Timer callback for publish
    def timer_callback(self):
        self.joint_state_publisher()

    '''
    Indy follow joint trajectory 
    '''
    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request :)')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request :(')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        print('FollowJointTrajectory callback...')
        # last_time = self.get_clock().now()
        goal = goal_handle.request.trajectory.points.copy()

        # download planned path from ros moveit
        self.joint_state_list = []
        if goal:
            self.joint_state_list = [p.positions for p in goal]
        else:
            self.grpc_stub.StopRobot(IndyRosInterface.Empty())

        # Do something for OP_IDLE state
        if self.joint_state_list:

            #--------------reduce the trajectory points----------
            new_array = [self.joint_state_list[0]]
            percentage = 95 # percentage of reduced point
            number = int(len(self.joint_state_list) * (1 - (percentage/100))) - 2
            if number > 0:
                distance = int((len(self.joint_state_list) - 2) / (number + 1))
                for i in range(1, number + 1):
                    new_array.append(self.joint_state_list[i * distance])
            new_array.append(self.joint_state_list[-1])
            #---------------------------------------------------

            result = FollowJointTrajectory.Result()
            feedback_msg = FollowJointTrajectory.Feedback()

            current_robot_status = self.grpc_stub.GetRobotState(IndyRosInterface.Empty())
            error_messages = {
                IndyRosInterface.OP_MOVING: "ROBOT IS BUSY",
                IndyRosInterface.OP_RECOVER_SOFT: "ROBOT IS IN RECOVER STATE",
                IndyRosInterface.OP_COLLISION: "ROBOT IS IN COLLISION STATE",
                IndyRosInterface.OP_TEACHING: "ROBOT IS IN TEACHING MODE",
                IndyRosInterface.OP_VIOLATE: "ROBOT IS IN VIOLATION STATE"
            }

            if current_robot_status.state in error_messages:
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = error_messages[current_robot_status.state]
                return result

            waypoints = []
            # desired_position = new_array[-1]

            for j_pos in new_array:
                waypoints.append(IndyRosInterface.WayPoint(jpos=j_pos, 
                                base_type=IndyRosInterface.JOINT_BASE_TYPE_ABSOLUTE, 
                                blending_rad=self.blend))
            self.grpc_stub.MoveJointWaypoint(IndyRosInterface.MoveJointWaypointReq(way_point=waypoints))
            time.sleep(0.2)

            while current_robot_status.state != IndyRosInterface.OP_MOVING:
                current_robot_status = self.grpc_stub.GetRobotState(IndyRosInterface.Empty())
                self.get_logger().info('Wait for robot start to move...')
                time.sleep(0.1)

            self.get_logger().info('robot is moving...')
            while current_robot_status.state != IndyRosInterface.OP_IDLE:

                if goal_handle.is_cancel_requested:
                    self.grpc_stub.StopRobot(IndyRosInterface.Empty())
                    goal_handle.canceled()
                    return result

                # print("self.joint_state_feedback.positions: ", self.joint_state_feedback.positions)
                feedback_msg.desired.positions = self.joint_state_feedback.positions
                feedback_msg.actual.positions = self.joint_state_feedback.positions
                goal_handle.publish_feedback(feedback_msg)

                # self.get_logger().info('robot is moving...')
                time.sleep(0.1)
                current_robot_status = self.grpc_stub.GetRobotState(IndyRosInterface.Empty())

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)

    with IndyROSConnector() as indy_driver:
        executor = MultiThreadedExecutor()
        rclpy.spin(indy_driver, executor=executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
