import sys
import json
import grpc
import math
import time

sys.path.append("../gRPCServerGenPython")
sys.path.append("../utils")

from IndygRPCTask_pb2_grpc import GRPCIndyTaskStub
from IndygRPCTask_pb2 import *


class IndyMaster:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC        
        self.channel = grpc.insecure_channel('%s:50052'%step_ip)
        self.stub = GRPCIndyTaskStub(self.channel)
    
    ## API    
    # Joint Servo Commands
    def set_sync_mode(self, mode):        
        return bool(self.stub.SetSyncMode(Value(boolVal=mode)).boolVal)
    
    def stop_emergency(self):
        return self.stub.StopEmergency(Empty())
    
    def reset_robot(self):
        return self.stub.ResetRobot(Empty())
    
    def set_servo(self, joint):        
        return self.stub.SetServo(JointBool(value=joint))
    
    def set_brake(self, joint):
        return self.stub.SetBrake(JointBool(value=joint))
    
    # Motion commands
    def stop_motion(self):
        return self.stub.StopMotion(Empty())
    
    def execute_move(self, move_name):
        return self.stub.ExecuteMove(Value(strVal=move_name))    
    
    def go_home(self):
        return self.stub.GoHome(Empty())    
    
    def go_zero(self):
        return self.stub.GoZero(Empty())    
    
    def joint_move_to(self, pos):
        return self.stub.JointMoveTo(JointFloat(joint_float=pos))
    
    def joint_move_by(self, pos):
        return self.stub.JointMoveBy(JointFloat(joint_float=pos))
    
    def task_move_to(self, pos):
        return self.stub.TaskMoveTo(TaskFloat(task_float=pos))
    
    def task_move_by(self, pos):
        return self.stub.TaskMoveBy(TaskFloat(task_float=pos))    
    
    # Teaching mode
    def direct_teaching(self, value):
        return self.stub.DirectTeaching(Value(boolVal=value))
    
    # Program control
    def start_current_program(self):
        return self.stub.StartCurrentProgram(Empty())
    
    def pause_current_program(self):
        return self.stub.PauseCurrentProgram(Empty())
    
    def resume_current_program(self):
        return self.stub.ResumeCurrentProgram(Empty())
    
    def stop_current_program(self):
        return self.stub.StopCurrentProgram(Empty())
    
    def start_default_program(self):
        return self.stub.StartDefaultProgram(Empty())
    
    def set_default_program(self, idx):
        return self.stub.SetDefaultProgram(Value(intVal=idx))
    
    def get_default_program_idx(self):
        return int(self.stub.GetDefaultProgramIdx(Empty()).intVal)
    
    # Robot status
    def get_robot_status(self):
        status = self.stub.GetRobotStatus(Empty())
        res = {'task_run': status.status[0],
               'ready': status.status[1],
               'violation': status.status[2],
               'collision': status.status[3],
               'emergency': status.status[4],               
               'busy': status.status[5],
               'movedone': status.status[6],
               'home': status.status[7],
               'zero': status.status[8],
               'resetting': status.status[9],
               'direct_teaching': status.status[10],
               'teaching': status.status[11],
               'program_state': status.status[12],
               'conty_connect': status.status[13]}
        return res
    
    # Set robot properties
    def set_default_tcp(self, tcp):
        return self.stub.SetDefaultTCP(TaskFloat(task_float=tcp))
    
    def reset_default_tcp(self):
        return self.stub.ResetDefaultTCP(Empty())
    
    def set_tcp_compensation(self, tcp):
        return self.stub.SetTCPCompensation(TaskFloat(task_float=tcp))
    
    def reset_tcp_compensation(self):
        return self.stub.ResetTCPCompensation(Empty())
    
    def set_reference_frame(self, ref):
        return self.stub.SetReferenceFrame(TaskFloat(task_float=ref))
    
    def reset_reference_frame(self):
        return self.stub.ResetReferenceFrame(Empty())
    
    def set_collision_level(self, level):
        return self.stub.SetCollisionLevel(Value(intVal=level))
    
    def set_joint_vel_level(self, level):
        return self.stub.SetJointVelLevel(Value(intVal=level))
    
    def set_task_vel_level(self, level):
        return self.stub.SetTaskVelLevel(Value(intVal=level))
    
    def set_joint_acc_level(self, level):
        return self.stub.SetJointAccLevel(Value(intVal=level))
    
    def set_task_acc_level(self, level):
        return self.stub.SetTaskAccLevel(Value(intVal=level))
    
    def set_reduced_speed_mode(self, mode):
        return self.stub.SetReducedSpeedMode(Value(boolVal=mode))
    
    def set_reduced_speed_ratio(self, ratio):
        return self.stub.SetReducedSpeedRatio(Value(floatVal=ratio))
    
    # Get robot properties
    def get_default_tcp(self):
        return list(self.stub.GetDefaultTCP(Empty()).task_float)
    
    def get_tcp_compensation(self):
        return list(self.stub.GetTCPCompensation(Empty()).task_float)
    
    def get_reference_frame(self):
        return list(self.stub.GetReferenceFrame(Empty()).task_float)
    
    def get_collision_level(self):
        return int(self.stub.GetCollisionLevel(Empty()).intVal)
    
    def get_joint_vel_level(self):
        return int(self.stub.GetJointVelLevel(Empty()).intVal)
    
    def get_task_vel_level(self):
        return int(self.stub.GetTaskVelLevel(Empty()).intVal)
    
    def get_joint_acc_level(self):
        return int(self.stub.GetJointAccLevel(Empty()).intVal)
    
    def get_task_acc_level(self):
        return int(self.stub.GetTaskAccLevel(Empty()).intVal)
    
    def get_reduced_speed_mode(self):
        return bool(self.stub.GetReducedSpeedMode(Empty()).boolVal)
    
    def get_reduced_speed_ratio(self):
        return float(self.stub.GetReducedSpeedRatio(Empty()).floatVal)
    
    def get_robot_running_time(self):
        return float(self.stub.GetRobotRunningTime(Empty()).doubleVal)
    
    def get_cmode(self):
        return int(self.stub.GetCmode(Empty()).intVal)
    
    def get_servo_state(self):
        return list(self.stub.GetServoState(Empty()).value)
    
    # Get robot data
    def get_joint_pos(self):
        return list(self.stub.GetJointPos(Empty()).joint_float)
    
    def get_joint_vel(self):
        return list(self.stub.GetJointVel(Empty()).joint_float)
    
    def get_task_pos(self):
        return list(self.stub.GetTaskPos(Empty()).task_float)
    
    def get_task_vel(self):
        return list(self.stub.GetTaskVel(Empty()).task_float)
    
    def get_control_torque(self):
        return list(self.stub.GetControlTorque(Empty()).joint_float)
    
    def get_inv_kin(self, target_p, init_q):
        joint = JointFloat(joint_float=init_q)
        task = TaskFloat(task_float=target_p)
        return list(self.stub.GetInvKin(JointTask(task=task, joint=joint)).joint_float)
    
    # I/O control
    def get_di(self):
        return list(self.stub.GetDI(Empty()).boolVal)
    
    def set_do(self, idx, val):
        return self.stub.SetDO(IOControl(index=idx, boolVal=[val]))
    
    def get_ai(self):
        return list(self.stub.GetAI(Empty()).intVal)[0:2]
    
    def set_ao(self, idx, val):
        return self.stub.SetAO(IOControl(index=idx, intVal=[val]))
    
    def get_do(self):
        return list(self.stub.GetDO(Empty()).intVal)
    
    def get_ao(self):
        return list(self.stub.GetAO(Empty()).intVal)[0:2]
    
    # Endtool control
    def set_endtool_do(self, idx, val):
        return self.stub.SetEndToolDO(Value(intVal=idx, boolVal=val))
    
    def get_endtool_do(self, idx):
        return int(self.stub.GetEndToolDO(Value(intVal=idx)).intVal)
    
    # F/T sensor interface
    def get_robot_ft_raw(self):
        return list(self.stub.GetRobotFTRaw(Empty()).intVal)
    
    def get_robot_ft(self):
        return list(self.stub.GetRobotFTTrans(Empty()).task_float)
    
    def get_cb_ft_raw(self):
        return list(self.stub.GetCBFTRaw(Empty()).intVal)
    
    def get_cb_ft(self):
        return list(self.stub.GetCBFTTrans(Empty()).task_float)
    
    # Direct variable
    def read_direct_variable(self, dv_type, dv_addr):        
        val = self.stub.ReadDirectVariable(RepeatInt(intVal=[dv_type, dv_addr]))
        if dv_type>=0 and dv_type<=3:
            return val.intVal
        elif dv_type==4 or dv_type==5:
            return val.floatVal
        elif dv_type==10:
            return val.intVal
    
    def write_direct_variable(self, dv_type, dv_addr, value):
        if dv_type==0:
            _type = 'B'
        elif dv_type==1:
            _type = 'W'
        elif dv_type==2:
            _type = 'I'
        elif dv_type==3:
            _type = 'L'
        elif dv_type==4:
            _type = 'F'
        elif dv_type==5:
            _type = 'D'
        elif dv_type==10:
            _type = 'M'        
        type_addr = _type + str(dv_addr).zfill(3)
        return self.stub.WriteDirectVariable(DirectVariable(type_address=type_addr, dv_value=str(value)))
    
    # Jog control
    def joint_jog(self, direction, vel_level):        
        return self.stub.JointJog(RepeatInt(intVal=[direction[0], direction[1], direction[2], direction[3], direction[4], 
                                                   direction[5], vel_level]))
    def task_jog(self, direction, vel_level, base):
        return self.stub.TaskJog(RepeatInt(intVal=[direction[0], direction[1], direction[2], direction[3], direction[4], 
                                                   direction[5], vel_level, base]))
    
    def stop_jog(self):
        return self.stub.StopJog(Empty())
    
    # Teleoperation
    def start_teleoperation(self):        
        return self.stub.StartTeleoperation(Empty())
    
    def stop_teleoperation(self):        
        return self.stub.StopTeleoperation(Empty())
    
    def update_teleoperation_traj(self, target_pos, recording):
        return self.stub.UpdateTeleoperationTraj(TeleopTask(x=target_pos[0], y=target_pos[1], z=target_pos[2], u=target_pos[3], v=target_pos[4], w=target_pos[5], recording=recording))
    
    def start_teleoperation_joint(self):
        return self.stub.StartTeleoperationJoint(Empty())
    
    def stop_teleoperation_joint(self):
        return self.stub.StopTeleoperationJoint(Empty())
    
    def update_teleoperation_traj_joint(self, target_joint_pos, time_const):
        return self.stub.UpdateTeleoperationTrajJoint(JointFloat(joint_float=[target_joint_pos[0], target_joint_pos[1], target_joint_pos[2], target_joint_pos[3], target_joint_pos[4], target_joint_pos[5], time_const]))
    