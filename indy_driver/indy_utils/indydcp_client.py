'''
Created on 2019. 6. 17.

@author: YJHeo
'''

'''
Edited on 2020. 7. 2.

@author: gwkim
'''

debugging = False

import socket
import sys
import os.path
import numpy as np
import json

from ctypes import *
from threading import Lock

###############################################################################
# Robot S/W version                                                           #
###############################################################################
SW_Version = "v2.3.1"  # 2020. 01. 14.

#########################################################################
# Global indicator                                                      #
#########################################################################
GLOBAL_DICT = {'stop': True, 'pause': False}
# GLOBAL_STOP = True

###############################################################################
# Global variables                                                            #
###############################################################################
JOINT_DOF = 6

###############################################################################
# Robot Interface                                                             #
###############################################################################
SIZE_HEADER = 52
SIZE_COMMAND = 4
SIZE_HEADER_COMMAND = 56
SIZE_DATA_TCP_MAX = 200
SIZE_DATA_MAX = 200
SIZE_DATA_ASCII_MAX = 32
SIZE_PACKET = 256

###############################################################################
# Robot Type                                                                  #
###############################################################################
ROBOT_INDY7 = "NRMK-Indy7"
ROBOT_INDYRP2 = "NRMK-IndyRP2"
ROBOT_INDY12 = "NRMK-Indy12"

# Deprecated
ROBOT_INDYRP = "NRMK-IndyRP"
ROBOT_INDY3 = "NRMK-Indy3"
ROBOT_INDY5 = "NRMK-Indy5"
ROBOT_INDY10 = "NRMK-Indy10"
ROBOT_INDY15 = "NRMK-Indy15"
ROBOT_OPTI5 = "NRMK-OPTi5"
ROBOT_OPTI10 = "NRMK-OPTi10"


###############################################################################
# C-type Data                                                                 #
###############################################################################
class HeaderCommandStruct(Structure):
    _pack_ = 1
    _fields_ = [("robotName", c_ubyte * 20),
                ("robotVersion", c_ubyte * 12),
                ("stepInfo", c_ubyte),
                ("sof", c_ubyte),
                ("invokeId", c_uint32),
                ("dataSize", c_uint32),
                ("status", c_uint32),
                ("reserved", c_ubyte * 6),
                ("cmdId", c_uint32)]


class HeaderCommand(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("val", HeaderCommandStruct)]


class Data(Union):
    _fields_ = [("byte", c_ubyte * SIZE_DATA_TCP_MAX),
                ("asciiStr", c_ubyte * (SIZE_DATA_ASCII_MAX + 1)),
                ("str", c_ubyte * 200),
                ("charVal", c_ubyte),
                ("boolVal", c_byte),
                ("shortVal", c_uint16),
                ("intVal", c_int32),
                ("floatVal", c_float),
                ("doubleVal", c_double),
                ("byteVal", c_ubyte),
                ("wordVal", c_ubyte * 2),
                ("uwordVal", c_ubyte * 2),
                ("dwordVal", c_ubyte * 4),
                ("lwordVal", c_ubyte * 8),
                ("bool6dArr", c_ubyte * 6),
                ("bool7dArr", c_ubyte * 7),
                ("boolArr", c_ubyte * 200),
                ("char2dArr", c_ubyte * 2),
                ("char3dArr", c_ubyte * 3),
                ("char6dArr", c_ubyte * 6),
                ("char7dArr", c_ubyte * 7),
                ("charArr", c_ubyte * 200),
                ("int2dArr", c_int32 * 2),
                ("int3dArr", c_int32 * 3),
                ("int6dArr", c_int32 * 6),
                ("int7dArr", c_int32 * 7),
                ("intArr", c_int32 * 50),
                ("float3dArr", c_float * 3),
                ("float6dArr", c_float * 6),
                ("float7dArr", c_float * 7),
                ("floatArr", c_float * 50),
                ("double3dArr", c_double * 3),
                ("double6dArr", c_double * 6),
                ("double7dArr", c_double * 7),
                ("doubleArr", c_double * 50),
                ("byteArr", c_ubyte * 200),
                ("wordArr", c_ubyte * 2 * 100),
                ("uwordArr", c_ubyte * 2 * 100),
                ("dwordArr", c_ubyte * 4 * 50),
                ("lwordArr", c_ubyte * 8 * 25)]


class Packet(Union):
    _fields_ = [("header", HeaderCommand),
                ("data", Data)]


class RobotStatus(Structure):
    _fields_ = [("is_robot_running", c_ubyte),
                ("is_robot_ready", c_ubyte),
                ("is_emergency_stop", c_ubyte),
                ("is_collided", c_ubyte),
                ("is_error_state", c_ubyte),
                ("is_busy", c_ubyte),
                ("is_move_finished", c_ubyte),
                ("is_home", c_ubyte),
                ("is_zero", c_ubyte),
                ("is_in_resetting", c_ubyte),
                ("is_direct_teaching_mode", c_ubyte),
                ("is_teaching_mode", c_ubyte),
                ("is_program_running", c_ubyte),
                ("is_program_paused", c_ubyte),
                ("is_conty_connected", c_ubyte)]


class DIO(Structure):
    _fields_ = [("channel", c_uint32),
                ("value", c_ubyte)]


#########################################################################
# Command                                                               #
#########################################################################
CMD_CHECK = 0
CMD_EMERGENCY_STOP = 1
CMD_RESET_ROBOT = 2
CMD_SET_SERVO = 3
CMD_SET_BRAKE = 4
CMD_STOP = 5
CMD_MOVE = 6
CMD_MOVE_HOME = 7
CMD_MOVE_ZERO = 8
CMD_JOINT_MOVE_TO = 9
CMD_JOINT_MOVE_BY = 10
CMD_TASK_MOVE_TO = 11
CMD_TASK_MOVE_BY = 12
CMD_TASK_MOVE_VELOCITY = 13

CMD_START_CURRENT_PROGRAM = 14
CMD_PAUSE_CURRENT_PROGRAM = 15
CMD_RESUME_CURRENT_PROGRAM = 16
CMD_STOP_CURRENT_PROGRAM = 17
CMD_START_DEFAULT_PROGRAM = 18
CMD_REGISTER_DEFAULT_PROGRAM_IDX = 19
CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX = 20

CMD_IS_ROBOT_RUNNING = 30
CMD_IS_READY = 31
CMD_IS_EMG = 32
CMD_IS_COLLIDED = 33
CMD_IS_ERR = 34
CMD_IS_BUSY = 35
CMD_IS_MOVE_FINISHED = 36
CMD_IS_HOME = 37
CMD_IS_ZERO = 38
CMD_IS_IN_RESETTING = 39
CMD_IS_DIRECT_TEACHING = 60
CMD_IS_TEACHING = 61
CMD_IS_PROGRAM_RUNNING = 62
CMD_IS_PROGRAM_PAUSED = 63
CMD_IS_CONTY_CONNECTED = 64

CMD_CHANGE_DIRECT_TEACHING = 80
CMD_FINISH_DIRECT_TEACHING = 81

CMD_JOINT_PUSH_BACK_WAYPOINT_SET = 90
CMD_JOINT_POP_BACK_WAYPOINT_SET = 91
CMD_JOINT_CLEAR_WAYPOINT_SET = 92
CMD_JOINT_EXECUTE_WAYPOINT_SET = 94
CMD_TASK_PUSH_BACK_WAYPOINT_SET = 95
CMD_TASK_POP_BACK_WAYPOINT_SET = 96
CMD_TASK_CLEAR_WAYPOINT_SET = 97
CMD_TASK_EXECUTE_WAYPOINT_SET = 99

CMD_SET_DEFAULT_TCP = 100
CMD_RESET_DEFAULT_TCP = 101
CMD_SET_COMP_TCP = 102
CMD_RESET_COMP_TCP = 103
CMD_SET_REFFRAME = 104
CMD_RESET_REFFRAME = 105
CMD_SET_COLLISION_LEVEL = 106
CMD_SET_JOINT_BOUNDARY = 107
CMD_SET_TASK_BOUNDARY = 108
CMD_SET_JOINT_WTIME = 111
CMD_SET_TASK_WTIME = 112
CMD_SET_TASK_CMODE = 113
CMD_SET_JOINT_BLEND_RADIUS = 116
CMD_SET_TASK_BLEND_RADIUS = 117

CMD_SET_REDUCED_MODE = 130  # not released
CMD_SET_REDUCED_SPEED_RATIO = 131  # not released
CMD_GET_REDUCED_MODE = 230  # not released
CMD_GET_REDUCED_SPEED_RATIO = 231  # not released

CMD_GET_DEFAULT_TCP = 200
CMD_GET_COMP_TCP = 201
CMD_GET_REFFRAME = 202
CMD_GET_COLLISION_LEVEL = 203
CMD_GET_JOINT_BOUNDARY = 204
CMD_GET_TASK_BOUNDARY = 205
CMD_GET_JOINT_WTIME = 208
CMD_GET_TASK_WTIME = 209
CMD_GET_TASK_CMODE = 210
CMD_GET_JOINT_BLEND_RADIUS = 213
CMD_GET_TASK_BLEND_RADIUS = 214

CMD_GET_RUNNING_TIME = 300
CMD_GET_CMODE = 301
CMD_GET_JOINT_STATE = 302
CMD_GET_JOINT_POSITION = 320
CMD_GET_JOINT_VELOCITY = 321
CMD_GET_TASK_POSITION = 322
CMD_GET_TASK_VELOCITY = 323
CMD_GET_TORQUE = 324
CMD_GET_INV_KIN = 325

CMD_GET_LAST_EMG_INFO = 380

CMD_GET_SMART_DI = 400
CMD_GET_SMART_DIS = 401
CMD_SET_SMART_DO = 402
CMD_SET_SMART_DOS = 403
CMD_GET_SMART_AI = 404
CMD_SET_SMART_AO = 405
CMD_GET_SMART_DO = 406
CMD_GET_SMART_DOS = 407
CMD_GET_SMART_AO = 408
CMD_SET_ENDTOOL_DO = 409  # v2.3.1 added
CMD_GET_ENDTOOL_DO = 410  # v2.3.1 added

CMD_GET_EXTIO_FTCAN_ROBOT_RAW = 420
CMD_GET_EXTIO_FTCAN_ROBOT_TRANS = 421
CMD_GET_EXTIO_FTCAN_CB_RAW = 422
CMD_GET_EXTIO_FTCAN_CB_TRANS = 423

CMD_READ_DIRECT_VARIABLE = 460
CMD_READ_DIRECT_VARIABLES = 461
CMD_WRITE_DIRECT_VARIABLE = 462
CMD_WRITE_DIRECT_VARIABLES = 463

CMD_SET_SYNC_MODE = 700

CMD_FOR_EXTENDED = 800
CMD_FOR_STREAMING = 801

CMD_SEND_KEYCOMMAND = 9996
CMD_READ_MEMORY = 9997
CMD_WRITE_MEMORY = 9998
CMD_ERROR = 9999

#########################################################################
# Extended DCP command                                                  #
#########################################################################
EXT_CMD_MOVE_TRAJ_BY_DATA = 1
EXT_CMD_MOVE_TRAJ_BY_TXT_DATA = 2
EXT_CMD_MOVE_TRAJ_BY_FILE = 3
EXT_CMD_MOVE_TRAJ_BY_TXT_FILE = 4

EXT_CMD_JMOVE_ABS_WAYPOINT_SET = 11
EXT_CMD_TMOVE_ABS_WAYPOINT_SET = 12

EXT_CMD_SET_JSON_PROG = 21
EXT_CMD_SET_JSON_PROG_START = 22

#########################################################################
# Error code                                                            #
#########################################################################
ERR_NONE = 0
ERR_NO_MATCHED_ROBOT = 1
ERR_NO_MATCHED_STEP = 2
ERR_HEADER_FORMAT = 4
ERR_OVER_DATA_SIZE = 5
ERR_NOT_SUPPORT_COMMAND = 6
ERR_UNKNOWN_COMMAND = 7
ERR_UNKNOWN_DATA = 8
ERR_PROCESS_FAILED = 9
ERR_PARSE_FAILED = 10
ERR_NO_MATCHED_PARAMETER = 11
ERR_NO_MATCHED_DATA_SIZE = 12
ERR_WRONG_ASCII_FORMAT = 13
ERR_ROBOT_MOVING_STATE = 14
ERR_ROBOT_PROGRAM_RUNNING = 15
ERR_ROBOT_MOVE_FAILED = 16
ERR_NO_DEFAULT_PROGRAM = 17
ERR_NO_CURRENT_PROGRAM = 18
ERR_CURRENT_PROGRAM_STATE = 19
ERR_EMG_STATE = 20
ERR_ROBOT_STATE = 21
ERR_ROBOT_PROGRAM_LOAD_FAILED = 22
ERR_DIRECT_VARIABLE_INVALID_ADDRESS = 23
ERR_DIRECT_VARIABLE_INVALID_FORMAT = 24
ERR_DIRECT_VARIABLE_REFNUM_LIMIT = 25
ERR_CONNECTION_EXCEPTION = 600
ERR_CONNECTION_TIMEOUT = 601


def err_to_string(err_cmd):
    return {ERR_NONE: "ErrorCode {}: No Error".format(err_cmd),
            ERR_NO_MATCHED_ROBOT: "ErrorCode {}: Not matched robot".format(err_cmd),
            ERR_NO_MATCHED_STEP: "ErrorCode {}: Not matched step".format(err_cmd),
            ERR_HEADER_FORMAT: "ErrorCode {}: Invalid header format".format(err_cmd),
            ERR_OVER_DATA_SIZE: "ErrorCode {}: Over data size".format(err_cmd),
            ERR_NOT_SUPPORT_COMMAND: "ErrorCode {}: Unsupported command".format(err_cmd),
            ERR_UNKNOWN_COMMAND: "ErrorCode {}: Unknown command".format(err_cmd),
            ERR_UNKNOWN_DATA: "ErrorCode {}: Unknown data".format(err_cmd),
            ERR_PROCESS_FAILED: "ErrorCode {}: Process fail".format(err_cmd),
            ERR_PARSE_FAILED: "ErrorCode {}: Parsing fail (data error)".format(err_cmd),
            ERR_NO_MATCHED_PARAMETER: "ErrorCode {}: Not matched data type".format(err_cmd),
            ERR_NO_MATCHED_DATA_SIZE: "ErrorCode {}: Not matched data size ".format(err_cmd),
            # ERR_WRONG_ASCII_FORMAT: "ErrorCode {}: ".format(err_cmd),
            ERR_ROBOT_MOVING_STATE: "ErrorCode {}: Robot is moving".format(err_cmd),
            ERR_ROBOT_PROGRAM_RUNNING: "ErrorCode {}: Robot program is running".format(err_cmd),
            ERR_ROBOT_MOVE_FAILED: "ErrorCode {}: Move fail".format(err_cmd),
            ERR_NO_DEFAULT_PROGRAM: "ErrorCode {}: No default program".format(err_cmd),
            ERR_NO_CURRENT_PROGRAM: "ErrorCode {}: No loaded program".format(err_cmd),
            ERR_CURRENT_PROGRAM_STATE: "ErrorCode {}: No proper program state".format(err_cmd),
            ERR_EMG_STATE: "ErrorCode {}: Robot is emergency state".format(err_cmd),
            ERR_ROBOT_STATE: "ErrorCode {}: Not proper robot state".format(err_cmd),
            ERR_ROBOT_PROGRAM_LOAD_FAILED: "ErrorCode {}: Program load fail".format(err_cmd),
            ERR_DIRECT_VARIABLE_INVALID_ADDRESS: "ErrorCode {}: Invalid direct variable address".format(err_cmd),
            ERR_DIRECT_VARIABLE_INVALID_FORMAT: "ErrorCode {}: Invalid direct variable format".format(err_cmd),
            ERR_DIRECT_VARIABLE_REFNUM_LIMIT: "ErrorCode {}: Limit of direct variable size".format(err_cmd)}.get(
        err_cmd, "None")


#########################################################################
# Header Status Bit                                                     #
#########################################################################
HEADER_STATUS_BIT_TASK_RUNNING = 0x80000000  # 0b 1000 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ROBOT_READY = 0x40000000  # 0b 0100 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_EMG_STOPPED = 0x20000000  # 0b 0010 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_COLLIDED = 0x10000000  # 0b 0001 0000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ERR_STATE = 0x08000000  # 0b 0000 1000 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_BUSY = 0x04000000  # 0b 0000 0100 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_MOVE_FINISHED = 0x02000000  # 0b 0000 0010 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_HOME = 0x01000000  # 0b 0000 0001 0000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_ZERO = 0x00800000  # 0b 0000 0000 1000 0000 0000 0000 0000 0000
HEADER_STATUS_BIT_IN_RESETTING = 0x00400000  # 0b 0000 0000 0100 0000 0000 0000 0000 0000

HEADER_STATUS_BIT_DIRECT_TEACHING = 0x00000080  # 0b 0000 0000 0000 0000 0000 0000 1000 0000
HEADER_STATUS_BIT_TEACHING = 0x00000040  # 0b 0000 0000 0000 0000 0000 0000 0100 0000
HEADER_STATUS_BIT_PROGRAM_RUNNING = 0x00000020  # 0b 0000 0000 0000 0000 0000 0000 0010 0000
HEADER_STATUS_BIT_PROGRAM_PAUSED = 0x00000010  # 0b 0000 0000 0000 0000 0000 0000 0001 0000
HEADER_STATUS_BIT_CONTY_CONNECTED = 0x00000008  # 0b 0000 0000 0000 0000 0000 0000 0000 1000
#########################################################################
# DirectVariableType                                                    #
#########################################################################
DIRECT_VAR_TYPE_ERROR = -1
DIRECT_VAR_TYPE_BYTE = 0
DIRECT_VAR_TYPE_WORD = 1
DIRECT_VAR_TYPE_DWORD = 2
DIRECT_VAR_TYPE_LWORD = 3
DIRECT_VAR_TYPE_FLOAT = 4
DIRECT_VAR_TYPE_DFLOAT = 5
DIRECT_VAR_TYPE_MODBUS_REG = 10


###############################################################################
# Debug                                                                       #
###############################################################################
def dump_buf(msg, buf, length):
    if debugging:
        print(msg)
        for i in range(0, length):
            # print(i, end=' - ')
            print(buf[i])


###############################################################################
# Decorators                                                                  #
###############################################################################
def socket_connect(func):
    def decorated(*args, **kwargs):
        args[0].lock.acquire()
        # args[0].connect()
        func_out = func(*args, **kwargs)
        # args[0].disconnect()
        args[0].lock.release()
        return func_out

    return decorated


# gwkim

def tcp_command(cmd, response_type=None):
    def decorate(func):
        def decorated(*args):
            global JOINT_DOF
            _req_data = func(*args)
            if _req_data is None:
                error_code, _res_data, _ = args[0]._handle_command(cmd)
            else:
                error_code, _res_data, _ = args[0]._handle_command(cmd, _req_data[0], _req_data[1])

            # print("OKKKKKKKKKKKKKKKKKKKKKK")
            # print(_req_data)
            # print(error_code)
            # print(_res_data)

            if error_code:
                return error_code

            if response_type == 'jointArr':
                if JOINT_DOF == 6:
                    return np.array(_res_data.double6dArr).tolist()
                else:
                    return np.array(_res_data.double7dArr).tolist()
            elif response_type is not None:
                return np.array(eval('_res_data.' + response_type)).tolist()
            else:
                return None

        return decorated

    return decorate


###############################################################################
# Indy Client Class                                                           #
###############################################################################
class IndyDCPClient:
    def __init__(self, server_ip, robot_name, robot_version=""):
        global JOINT_DOF

        self.__server_port = 6066       #6066
        self.__sof_server = 0x12
        self.__sof_client = 0x34
        self.__step_ver = 0x02
        self.__lock = Lock()
        self.lock = self.__lock

        self.sock_fd = socket.socket()

        self.time_out = 10
        self.v_invokeId = 0

        self.server_ip = server_ip
        self.robot_name = robot_name
        self.robot_version = robot_version

        JOINT_DOF = 7 if self.robot_name == ROBOT_INDYRP2 else 6

        self.robot_status = RobotStatus()

    def connect(self):
        # self.__lock.acquire()
        self.sock_fd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.sock_fd.connect((self.server_ip, self.__server_port))
        except socket.error as e:
            print("Socket connection error: {}".format(e))
            self.sock_fd.close()
            # self.__lock.release()
            return False
        else:
            if True:
                print("Connect: Server IP ({ser_ip})".format(ser_ip=self.server_ip))
                # self.__lock.release()
            return True

    def disconnect(self):
        self.sock_fd.close()
        # self.__lock.release()

    def shutdown(self):
        self.sock_fd.shutdown(socket.SHUT_RDWR)
        print("Shut down")

    def set_timeout_sec(self, time_out):
        if time_out < 0:
            print("Invalid time out setting: {}<0".format(time_out))
        self.time_out = time_out

    def _send_message(self, buf, size):
        dump_buf("SendBytes: ", buf, size)
        total_sent = 0
        while total_sent < size:
            self.sock_fd.settimeout(self.time_out)
            sent = self.sock_fd.send(buf[total_sent:size])
            if sent == -1:
                print('Error: sent == -1')
                return -1
            elif sent == 0:
                # self.__lock.release()
                print('Error: sent == 0')
                return -1
            total_sent = total_sent + sent
        return 0

    def _recv_message(self, buf, size):
        chunks = []
        bytes_recd = 0
        while bytes_recd < size:
            self.sock_fd.settimeout(self.time_out)
            chunk = self.sock_fd.recv(size - bytes_recd)
            if chunk == b'':
                print('Error: receive error')
                memset(buf, 0, sizeof(buf))
                # self.__lock.release()
                self.shutdown()
                return -1
            chunks.append(chunk)
            if (bytes_recd + len(chunk)) > sizeof(buf):
                break
            bytes_recd += len(chunk)
        data = b''.join(chunks)
        # print("recv data: ", data)
        memset(buf, 0, sizeof(buf))
        memmove(buf, data, len(data))
        return buf

    def check_header(self, req=HeaderCommand(), res=HeaderCommand(), err_code=ERR_NONE):
        req_robot_name = np.array(req.val.robotName).tostring().decode('utf-8')
        res_robot_name = np.array(res.val.robotName).tostring().decode('utf-8')
        if req_robot_name != res_robot_name:
            print("Header check fail (robotName): Request {_req}, Response {_res}".format(_req=req_robot_name,
                                                                                          _res=res_robot_name))
        if req.val.stepInfo != res.val.stepInfo:
            print("Header check fail (stepInfo): Request {_req}, Response {_res}".format(_req=req.val.stepInfo,
                                                                                         _res=res.val.stepInfo))
        if req.val.invokeId != res.val.invokeId:
            print("Header check fail (invokeId): Request {_req}, Response {_res}".format(_req=req.val.invokeId,
                                                                                         _res=res.val.invokeId))
        if res.val.sof != self.__sof_server:
            print("Header check fail (sof): Request {_req}, Response {_res}".format(_req=self.__sof_server,
                                                                                    _res=res.val.sof))
        if req.val.cmdId != res.val.cmdId:
            print("Header check fail (cmdId): Request {_req}, Response {_res}".format(_req=req.val.cmdId,
                                                                                      _res=res.val.cmdId))
        if res.val.cmdId == CMD_ERROR:
            print(err_to_string(err_code))
            return err_code
        return ERR_NONE

    def parse_robot_status(self, status):
        # print(status)
        status_str = bin(status).lstrip('0b')
        # print (status_str)
        self.robot_status.is_robot_running = int(status_str[0])
        self.robot_status.is_robot_ready = int(status_str[1])
        self.robot_status.is_emergency_stop = int(status_str[2])
        self.robot_status.is_collided = int(status_str[3])
        self.robot_status.is_error_state = int(status_str[4])
        self.robot_status.is_busy = int(status_str[5])
        self.robot_status.is_move_finished = int(status_str[6])
        self.robot_status.is_home = int(status_str[7])
        self.robot_status.is_zero = int(status_str[8])
        self.robot_status.is_in_resetting = int(status_str[9])
        self.robot_status.is_teaching_mode = int(status_str[25])
        self.robot_status.is_direct_teaching_mode = int(status_str[24])
        self.robot_status.is_program_running = int(status_str[26])
        self.robot_status.is_program_paused = int(status_str[27])
        self.robot_status.is_conty_connected = int(status_str[28])

    @socket_connect
    def _handle_command(self, cmd, req_data=Data(), req_data_size=0):
        write_buffer = (c_char * 1024)()
        read_buffer = (c_char * 1024)()

        if req_data_size > SIZE_DATA_TCP_MAX or req_data_size < 0:
            self.disconnect()
            raise Exception("Request size is invalid {}: Disconnected".format(req_data_size))

        # Make header
        req_header = HeaderCommand()
        memset(req_header.byte, 0, sizeof(req_header.byte))

        b_str_robot_name = self.robot_name.encode('ascii')
        memmove(req_header.val.robotName, c_char_p(b_str_robot_name), len(self.robot_name))

        b_str_robot_ver = self.robot_version.encode('ascii')
        memmove(req_header.val.robotVersion, c_char_p(b_str_robot_ver), len(self.robot_version))

        req_header.val.stepInfo = self.__step_ver
        req_header.val.sof = self.__sof_client

        req_header.val.cmdId = cmd
        req_header.val.dataSize = req_data_size

        self.v_invokeId += 1
        req_header.val.invokeId = self.v_invokeId

        # Send packet to socket
        memmove(write_buffer, req_header.byte, SIZE_HEADER_COMMAND)
        self._send_message(write_buffer, SIZE_HEADER_COMMAND)

        # print("send_message: ", write_buffer, " ", SIZE_HEADER_COMMAND)

        if req_data_size > 0:
            if hasattr(req_data, 'byte'):
                memmove(write_buffer, req_data.byte, req_data_size)
            else:
                memmove(write_buffer, req_data, req_data_size)  # For execute command move
            self._send_message(write_buffer, req_data_size)

        # Recv header from socket
        res_header = HeaderCommand()
        read_buffer = self._recv_message(read_buffer, SIZE_HEADER_COMMAND)
        memmove(res_header.byte, read_buffer, SIZE_HEADER_COMMAND)

        # Recv data from socket
        res_data = Data()

        # print("res_data: ", res_data)

        res_data_size = res_header.val.dataSize
        if res_data_size > SIZE_DATA_TCP_MAX or res_data_size < 0:
            print("Response data size is invalid {} (max: {}): Disconnected".format(res_data_size, SIZE_DATA_TCP_MAX))
            self.disconnect()
        elif res_data_size > 0:
            # print("read_buffer: ", read_buffer)
            # print("res_data_size: ", res_data_size)
            read_buffer = self._recv_message(read_buffer, res_data_size)
            # print("read_buffer 1: ", read_buffer)
            memmove(res_data.byte, read_buffer, res_data_size)
            # print("res_data.byte: ", res_data.byte)
            # print("res_data_size 1: ", res_data_size)

        # Check header and error
        error_code = self.check_header(req_header, res_header, res_data.intVal)

        # Get robot status from header's reserved field
        self.parse_robot_status(res_header.val.status)
        return error_code, res_data, res_data_size

    @socket_connect
    def _handle_extended_command(self, ext_cmd, req_ext_data, req_ext_data_size=0):
        ret = False

        write_buffer = (c_char * 1024)()
        read_buffer = (c_char * 1024)()

        if req_ext_data_size > sys.maxsize or req_ext_data_size < 0:
            self.disconnect()
            print("Send data size error")
        if req_ext_data_size > 0 and req_ext_data is None:
            print("Send data error: Null data")

        # Make request header
        req_header = HeaderCommand()
        memset(req_header.byte, 0, sizeof(req_header.byte))

        b_str_robot_name = self.robot_name.encode('ascii')
        memmove(req_header.val.robotName, c_char_p(b_str_robot_name), len(self.robot_name))

        b_str_robot_ver = self.robot_version.encode('ascii')
        memmove(req_header.val.robotVersion, c_char_p(b_str_robot_ver), len(self.robot_version))

        req_header.val.stepInfo = self.__step_ver
        req_header.val.sof = self.__sof_client

        req_header.val.cmdId = CMD_FOR_EXTENDED
        req_header.val.dataSize = 8

        self.v_invokeId += 1
        req_header.val.invokeId = self.v_invokeId

        # Make request data
        req_data = Data()
        req_data.int2dArr[0] = np.array(ext_cmd)
        req_data.int2dArr[1] = np.array(req_ext_data_size)
        req_data_size = req_header.val.dataSize

        # Send packet to socket
        memmove(write_buffer, req_header.byte, SIZE_HEADER_COMMAND)
        self._send_message(write_buffer, SIZE_HEADER_COMMAND)
        memmove(write_buffer, req_data.byte, req_data_size)
        self._send_message(write_buffer, req_data_size)

        # Send extended packet to socket
        if req_ext_data_size > 0:
            self._send_message(req_ext_data, req_ext_data_size)

        # Recv header from socket
        res_header = HeaderCommand()
        read_buffer = self._recv_message(read_buffer, SIZE_HEADER_COMMAND)
        memmove(res_header.byte, read_buffer, SIZE_HEADER_COMMAND)

        # Recv data from socket
        res_data = Data()
        res_data_size = res_header.val.dataSize
        if res_data_size > SIZE_DATA_TCP_MAX or res_data_size < 0:
            self.disconnect()
        elif res_data_size > 0:
            read_buffer = self._recv_message(read_buffer, res_data_size)
            memmove(res_data.byte, read_buffer, res_data_size)

        # Check header and error
        ret = self.check_header(req_header, res_header, res_data)

        # Recv extended data from socket
        res_ext_data = Data()
        res_ext_data_size = res_data.int2dArr[1]

        if res_ext_data_size < 0 or res_ext_data_size > sys.maxsize:
            self.disconnect()
            print("Recv data error: size")
        elif res_data.int2dArr[0] is not ext_cmd:
            self.disconnect()
            print("Recv data error: ext_cmd {}/{}".format(res_data.int2dArr[0], ext_cmd))
        if res_ext_data_size > 0:
            self._recv_message(res_ext_data, res_ext_data_size)

        if not ret:
            return ret
        else:
            return ret, res_data, res_data_size

    # gwkim
    ############################################################################
    ## Robot command function (Check all)                                     #
    ############################################################################
    @tcp_command(CMD_CHECK)
    def check(self):
        pass

        # Get robot status

    def get_robot_status(self):
        self.check()
        res = {
            'running': self.robot_status.is_robot_running,
            'ready': self.robot_status.is_robot_ready,
            'emergency': self.robot_status.is_emergency_stop,
            'collision': self.robot_status.is_collided,
            'error': self.robot_status.is_error_state,
            'busy': self.robot_status.is_busy,
            'movedone': self.robot_status.is_move_finished,
            'home': self.robot_status.is_home,
            'zero': self.robot_status.is_zero,
            'resetting': self.robot_status.is_in_resetting,
            'teaching': self.robot_status.is_teaching_mode,
            'direct_teaching': self.robot_status.is_direct_teaching_mode,
            'program_running': self.robot_status.is_program_running,
            'program_paused': self.robot_status.is_program_paused,
            'conty_connected': self.robot_status.is_conty_connected
        }
        return res

    @tcp_command(CMD_IS_CONTY_CONNECTED, 'boolVal')
    def is_conty_connected(self):
        pass

    @tcp_command(CMD_IS_ROBOT_RUNNING, 'boolVal')
    def is_robot_running(self):
        pass

    @tcp_command(CMD_IS_READY, 'boolVal')
    def is_robot_ready(self):
        pass

    @tcp_command(CMD_IS_EMG, 'boolVal')
    def is_emergency_stop(self):
        pass

    @tcp_command(CMD_IS_COLLIDED, 'boolVal')
    def is_collided(self):
        pass

    @tcp_command(CMD_IS_ERR, 'boolVal')
    def is_error_state(self):
        pass

    @tcp_command(CMD_IS_BUSY, 'boolVal')
    def is_busy(self):
        pass

    @tcp_command(CMD_IS_MOVE_FINISHED, 'boolVal')
    def is_move_finished(self):
        pass

    @tcp_command(CMD_IS_HOME, 'boolVal')
    def is_home(self):
        pass

    @tcp_command(CMD_IS_ZERO, 'boolVal')
    def is_zero(self):
        pass

    @tcp_command(CMD_IS_IN_RESETTING, 'boolVal')
    def is_in_resetting(self):
        pass

    @tcp_command(CMD_IS_DIRECT_TEACHING, 'boolVal')
    def is_direct_teaching_mode(self):
        pass

    @tcp_command(CMD_IS_TEACHING, 'boolVal')
    def is_teaching_mode(self):
        pass

    @tcp_command(CMD_IS_PROGRAM_RUNNING, 'boolVal')
    def is_program_running(self):
        pass

    @tcp_command(CMD_IS_PROGRAM_PAUSED, 'boolVal')
    def is_program_paused(self):
        pass

    @tcp_command(CMD_IS_CONTY_CONNECTED, 'boolVal')
    def is_conty_connected(self):
        pass

    # Program state
    def get_program_state(self):
        self.check()
        res = {'running': self.robot_status.is_program_running,
               'pause': self.robot_status.is_program_paused}
        return res

    # Reset/Stop
    @tcp_command(CMD_EMERGENCY_STOP)
    def stop_emergency(self):
        pass

    @tcp_command(CMD_RESET_ROBOT)
    def reset_robot(self):
        pass

    # Joint/Servo command
    @tcp_command(CMD_SET_SERVO)
    def set_servo(self, arr):
        data = Data()
        data_size = JOINT_DOF
        for i in range(JOINT_DOF):
            data.bool6dArr[i] = arr[i]
        return (data, data_size)

    @tcp_command(CMD_SET_BRAKE)
    def set_brake(self, arr):
        data = Data()
        data_size = JOINT_DOF
        for i in range(JOINT_DOF):
            data.bool6dArr[i] = arr[i]
        return (data, data_size)

    def direct_teaching(self, mode):
        if mode:
            self._handle_command(CMD_CHANGE_DIRECT_TEACHING)
        else:
            self._handle_command(CMD_FINISH_DIRECT_TEACHING)

    # Set global robot variables
    @tcp_command(CMD_SET_DEFAULT_TCP)
    def set_default_tcp(self, tcp):
        data = Data()
        data_size = 6 * 8
        for i in range(JOINT_DOF):
            data.double6dArr[i] = tcp[i]
        return (data, data_size)

    @tcp_command(CMD_RESET_DEFAULT_TCP)
    def reset_default_tcp(self):
        pass

    @tcp_command(CMD_SET_COMP_TCP)
    def set_tcp_comp(self, tcp):
        data = Data()
        data_size = 6 * 8
        for i in range(JOINT_DOF):
            data.double6dArr[i] = tcp[i]
        return (data, data_size)

    @tcp_command(CMD_RESET_COMP_TCP)
    def reset_tcp_compensation(self):
        pass

    @tcp_command(CMD_SET_REFFRAME)
    def set_reference_frame(self, ref):
        data = Data()
        data_size = 6 * 8
        for i in range(JOINT_DOF):
            data.double6dArr[i] = ref[i]
        return (data, data_size)

    @tcp_command(CMD_RESET_REFFRAME)
    def reset_reference_frame(self):
        pass

    @tcp_command(CMD_SET_COLLISION_LEVEL)
    def set_collision_level(self, level):
        data = Data()
        data_size = 4
        data.intVal = level
        return (data, data_size)

    @tcp_command(CMD_SET_JOINT_BOUNDARY)
    def set_joint_vel_level(self, level):
        data = Data()
        data_size = 4
        data.intVal = level
        return (data, data_size)

    @tcp_command(CMD_SET_TASK_BOUNDARY)
    def set_task_vel_level(self, level):
        data = Data()
        data_size = 4
        data.intVal = level
        return (data, data_size)

    @tcp_command(CMD_SET_JOINT_WTIME)
    def set_joint_waypoint_time(self, wp_time):
        data = Data()
        data_size = 8
        data.doubleVal = wp_time
        return (data, data_size)

    @tcp_command(CMD_SET_TASK_WTIME)
    def set_task_waypoint_time(self, wp_time):
        data = Data()
        data_size = 8
        data.doubleVal = wp_time
        return (data, data_size)

    @tcp_command(CMD_SET_TASK_CMODE)
    def set_task_base(self, mode):
        # 0: reference frame, 1: TCO
        data = Data()
        data_size = 4
        data.intVal = mode
        return (data, data_size)

    @tcp_command(CMD_SET_JOINT_BLEND_RADIUS)
    def set_joint_blend_radius(self, radius):
        data = Data()
        data_size = 8
        data.doubleVal = radius
        return (data, data_size)

    @tcp_command(CMD_SET_TASK_BLEND_RADIUS)
    def set_task_blend_radius(self, radius):
        data = Data()
        data_size = 8
        data.doubleVal = radius
        return (data, data_size)

    # Get global robot variables
    @tcp_command(CMD_GET_DEFAULT_TCP, 'double6dArr')
    def get_default_tcp(self):
        pass

    @tcp_command(CMD_GET_COMP_TCP, 'double6dArr')
    def get_tcp_comp(self):
        pass

    @tcp_command(CMD_GET_REFFRAME, 'double6dArr')
    def get_reference_frame(self):
        pass

    @tcp_command(CMD_GET_COLLISION_LEVEL, 'intVal')
    def get_collision_level(self):
        pass

    @tcp_command(CMD_GET_JOINT_BOUNDARY, 'intVal')
    def get_joint_vel_level(self):
        pass

    @tcp_command(CMD_GET_TASK_BOUNDARY, 'intVal')
    def get_task_vel_level(self):
        pass

    @tcp_command(CMD_GET_JOINT_WTIME, 'doubleVal')
    def get_joint_waypoint_time(self):
        pass

    @tcp_command(CMD_GET_TASK_WTIME, 'doubleVal')
    def get_task_waypoint_time(self):
        pass

    @tcp_command(CMD_GET_TASK_CMODE, 'intVal')
    def get_task_base(self):
        pass

    @tcp_command(CMD_GET_JOINT_BLEND_RADIUS, 'doubleVal')
    def get_joint_blend_radius(self):
        pass

    @tcp_command(CMD_GET_TASK_BLEND_RADIUS, 'doubleVal')
    def get_task_blend_radius(self):
        pass

    @tcp_command(CMD_GET_RUNNING_TIME, 'doubleVal')
    def get_robot_running_time(self):
        pass

    @tcp_command(CMD_GET_CMODE, 'intVal')
    def get_cmode(self):
        pass

    def get_servo_state(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_JOINT_STATE)
        if error_code:
            return error_code

        result = np.array(_res_data.charArr)
        servo_state = result[0:JOINT_DOF].tolist()
        brake_state = result[JOINT_DOF:2 * JOINT_DOF].tolist()
        return servo_state, brake_state

    # Not released
    @tcp_command(CMD_SET_REDUCED_MODE)
    def set_reduced_mode(self, mode):
        data = Data()
        data_size = 1
        data.boolVal = mode
        return (data, data_size)

    @tcp_command(CMD_SET_REDUCED_SPEED_RATIO)
    def set_reduced_speed_ratio(self, ratio):
        data = Data()
        data_size = 8
        data.doubleVal = ratio
        return (data, data_size)

    @tcp_command(CMD_GET_REDUCED_MODE, 'boolVal')
    def get_reduced_mode(self):
        pass

    @tcp_command(CMD_GET_REDUCED_SPEED_RATIO, 'doubleVal')
    def get_reduced_speed_ratio(self):
        pass

    # Get robot data
    @tcp_command(CMD_GET_JOINT_POSITION, "jointArr")
    def get_joint_pos(self):
        pass

    @tcp_command(CMD_GET_JOINT_VELOCITY, "jointArr")
    def get_joint_vel(self):
        pass

    @tcp_command(CMD_GET_TASK_POSITION, 'double6dArr')
    def get_task_pos(self):
        pass

    @tcp_command(CMD_GET_TASK_VELOCITY, 'double6dArr')
    def get_task_vel(self):
        pass

    @tcp_command(CMD_GET_TORQUE, 'jointArr')
    def get_control_torque(self):
        pass

    def get_last_emergency_info(self):
        # Check (TODO: represent meaning of results)
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_LAST_EMG_INFO)
        if error_code:
            return error_code
        else:
            ret_code = c_int32()
            ret_int_arr = (c_int32 * 3)()
            ret_double_arr = (c_double * 3)()

            memmove(addressof(ret_code), addressof(_res_data.byte), 4)
            memmove(addressof(ret_int_arr), addressof(_res_data.byte) + 4, 4 * 3)
            memmove(addressof(ret_double_arr), addressof(_res_data.byte) + 16, 8 * 3)

            return np.array(ret_code).tolist(), np.array(ret_int_arr).tolist(), np.array(ret_double_arr).tolist()

    # Motion command
    @tcp_command(CMD_STOP)
    def stop_motion(self):
        pass

    @tcp_command(CMD_MOVE_HOME)
    def go_home(self):
        pass

    @tcp_command(CMD_MOVE_ZERO)
    def go_zero(self):
        pass

    @tcp_command(CMD_JOINT_MOVE_TO)
    def joint_move_to(self, q):
        data = Data()
        data_size = JOINT_DOF * 8
        for i in range(JOINT_DOF):
            data.doubleArr[i] = q[i]
        return data, data_size

    @tcp_command(CMD_JOINT_MOVE_BY)
    def joint_move_by(self, q):
        data = Data()
        data_size = JOINT_DOF * 8
        for i in range(JOINT_DOF):
            data.doubleArr[i] = q[i]
        return data, data_size

    @tcp_command(CMD_TASK_MOVE_TO)
    def task_move_to(self, p):
        data = Data()
        data_size = 6 * 8
        for i in range(6):
            data.double6dArr[i] = p[i]
        return data, data_size

    @tcp_command(CMD_TASK_MOVE_VELOCITY)
    def task_move_vel(self, p):
        data = Data()
        data_size = 6 * 8
        for i in range(6):
            data.double6dArr[i] = p[i]
        return data, data_size

    @tcp_command(CMD_TASK_MOVE_BY)
    def task_move_by(self, p):
        data = Data()
        data_size = 6 * 8
        for i in range(6):
            data.double6dArr[i] = p[i]
        return (data, data_size)

    # Waypoint move
    @tcp_command(CMD_JOINT_PUSH_BACK_WAYPOINT_SET)
    def joint_waypoint_append(self, q, wp_type=0, blend_radius=0):
        # wp_type: 0 (absolute), 1 (relative joint)
        # blend_radius: 0 ~ 23 [deg]

        data = Data()
        data_size = (JOINT_DOF + 2) * 8
        data.doubleArr[0] = wp_type

        if blend_radius >= 3 and blend_radius <= 27:
            data.doubleArr[1] = blend_radius
        else:
            data.doubleArr[1] = 0

        for i in range(JOINT_DOF):
            data.doubleArr[i + 2] = q[i]

        return (data, data_size)

    @tcp_command(CMD_JOINT_POP_BACK_WAYPOINT_SET)
    def joint_waypoint_remove(self):
        pass

    @tcp_command(CMD_JOINT_CLEAR_WAYPOINT_SET)
    def joint_waypoint_clean(self):
        pass

    # gwkim
    @tcp_command(CMD_JOINT_EXECUTE_WAYPOINT_SET)
    def joint_waypoint_execute(self, policy=0, resume_time=2):
        # 0 : stop
        # 1 : auto resume
        # 2 : auto resume reverse
        # 3 : no detection
        # return : target waypoint number when robot is collided

        data = Data()
        data_size = 2 * 8

        data.doubleArr[0] = policy
        data.doubleArr[1] = resume_time

        return (data, data_size)

    @tcp_command(CMD_TASK_PUSH_BACK_WAYPOINT_SET)
    def task_waypoint_append(self, p, wp_type=0, blend_radius=0):
        # wp_type: 0 (absolute), 2 (relative task)
        # task_base = 0 (base reference), 1 (base tcp)
        # blend radius: 0.02 ~ 0.2 [m]

        data = Data()
        data_size = (6 + 2) * 8
        data.doubleArr[0] = wp_type

        if blend_radius >= 0.02 and blend_radius <= 0.2:
            data.doubleArr[1] = blend_radius
        else:
            data.doubleArr[1] = 0

        for i in range(6):
            data.doubleArr[i + 2] = p[i]

        return (data, data_size)

    @tcp_command(CMD_TASK_POP_BACK_WAYPOINT_SET)
    def task_waypoint_remove(self):
        pass

    @tcp_command(CMD_TASK_CLEAR_WAYPOINT_SET)
    def task_waypoint_clean(self):
        pass

    # gwkim
    @tcp_command(CMD_TASK_EXECUTE_WAYPOINT_SET)
    def task_waypoint_execute(self, policy=0, resume_time=2):
        # 0 : stop
        # 1 : auto resume
        # 2 : auto resume reverse
        # 3 : no detection
        # return : target waypoint number when robot is collided
        data = Data()
        data_size = 2 * 8

        data.doubleArr[0] = policy
        data.doubleArr[1] = resume_time

        return (data, data_size)

    # Conty's move command
    @tcp_command(CMD_MOVE)
    def execute_move(self, cmd_name):
        data = cmd_name.encode('ascii')
        data_size = len(cmd_name)
        return (data, data_size)

    # Program control
    @tcp_command(CMD_START_CURRENT_PROGRAM)
    def start_current_program(self):
        pass

    @tcp_command(CMD_PAUSE_CURRENT_PROGRAM)
    def pause_current_program(self):
        pass

    @tcp_command(CMD_RESUME_CURRENT_PROGRAM)
    def resume_current_program(self):
        pass

    @tcp_command(CMD_STOP_CURRENT_PROGRAM)
    def stop_current_program(self):
        pass

    @tcp_command(CMD_START_DEFAULT_PROGRAM)
    def start_default_program(self):
        pass

    @tcp_command(CMD_REGISTER_DEFAULT_PROGRAM_IDX)
    def set_default_program(self, idx):
        data = Data()
        data_size = 4
        data.intVal = idx
        return (data, data_size)

    @tcp_command(CMD_GET_REGISTERED_DEFAULT_PROGRAM_IDX, 'intVal')
    def get_default_program_idx(self):
        pass

    # Digital/Analog IO
    def get_di(self):
        error_code, _res_data, _res_data_size = self._handle_command(CMD_GET_SMART_DIS)
        if error_code:
            return error_code
        else:
            return np.array(_res_data.charArr).tolist()[0:32]

    @tcp_command(CMD_SET_SMART_DO)
    def set_do(self, idx, val):
        data = Data()
        data_size = 5

        memset(data.byte, 0, sizeof(data.byte))
        memmove(data.byte, pointer(c_int32(idx)), sizeof(c_int32))
        memmove(addressof(data.byte) + 4, pointer(c_ubyte(val)), sizeof(c_ubyte))

        return (data, data_size)

    @tcp_command(CMD_GET_SMART_DOS, 'charArr')
    def get_do(self):
        pass

    @tcp_command(CMD_GET_SMART_AI, 'intVal')
    def get_ai(self, idx):
        data = Data()
        data_size = 4
        data.intVal = idx
        return (data, data_size)

    @tcp_command(CMD_SET_SMART_AO)
    def set_ao(self, idx, val):
        data = Data()
        data_size = 8
        data.intArr[0] = idx
        data.intArr[1] = val
        return (data, data_size)

    @tcp_command(CMD_GET_SMART_AO, 'intVal')
    def get_ao(self, idx):
        data = Data()
        data_size = 4
        data.intVal = idx
        return (data, data_size)

    @tcp_command(CMD_SET_ENDTOOL_DO)
    def set_endtool_do(self, endtool_type, val):
        # endtool_type:
        # 0: NPN, 1: PNP, 2: Not use, 3: eModi
        data = Data()
        data_size = 5
        memset(data.byte, 0, sizeof(data.byte))
        memmove(data.byte, pointer(c_int32(endtool_type)), sizeof(c_int32))
        memmove(addressof(data.byte) + 4, pointer(c_ubyte(val)), sizeof(c_ubyte))

        return (data, data_size)

    @tcp_command(CMD_GET_ENDTOOL_DO, 'charVal')
    def get_endtool_do(self, type):
        data = Data()
        data_size = 4
        data.intVal = type
        return (data, data_size)

    # FT sensor interface
    @tcp_command(CMD_GET_EXTIO_FTCAN_ROBOT_RAW, 'int6dArr')
    def get_robot_ft_raw(self):
        pass

    @tcp_command(CMD_GET_EXTIO_FTCAN_ROBOT_TRANS, 'double6dArr')
    def get_robot_ft(self):
        pass

    @tcp_command(CMD_GET_EXTIO_FTCAN_CB_RAW, 'int6dArr')
    def get_cb_ft_raw(self):
        pass

    @tcp_command(CMD_GET_EXTIO_FTCAN_CB_TRANS, 'double6dArr')
    def get_cb_ft(self):
        pass

    # Direct variables
    def read_direct_variable(self, dv_type, dv_addr):
        _req_data = Data()
        _req_data_size = 8
        _req_data.int2dArr[0] = dv_type
        _req_data.int2dArr[1] = dv_addr

        error_code, _res_data, _res_data_size = self._handle_command(CMD_READ_DIRECT_VARIABLE, _req_data,
                                                                     _req_data_size)

        if not error_code:
            if dv_type == DIRECT_VAR_TYPE_BYTE:  # B
                if _res_data_size == 1:
                    return int(_res_data.byteVal)

            elif dv_type == DIRECT_VAR_TYPE_WORD:  # W
                if _res_data_size == 2:
                    val = np.array(_res_data.wordVal).tolist()
                    res = int.from_bytes(val, byteorder='little', signed=True)
                    return res

            elif dv_type == DIRECT_VAR_TYPE_DWORD:  # I
                if _res_data_size == 4:
                    val = np.array(_res_data.dwordVal).tolist()
                    res = int.from_bytes(val, byteorder='little', signed=True)
                    return res

            elif dv_type == DIRECT_VAR_TYPE_LWORD:  # L
                if _res_data_size == 8:
                    val = np.array(_res_data.lwordVal).tolist()
                    res = int.from_bytes(val, byteorder='little', signed=True)
                    return res

            elif dv_type == DIRECT_VAR_TYPE_FLOAT:  # F
                if _res_data_size == 4:
                    return np.array(_res_data.floatVal)

            elif dv_type == DIRECT_VAR_TYPE_DFLOAT:  # D
                if _res_data_size == 8:
                    return np.array(_res_data.doubleVal)

            elif dv_type == DIRECT_VAR_TYPE_MODBUS_REG:  # M
                if _res_data_size == 2:
                    val = np.array(_res_data.uwordVal).tolist()
                    res = int.from_bytes(val, byteorder='little', signed=False)
                    return res

            else:
                print("None matched type")
                return False
        else:
            return error_code

    def read_direct_variables(self, dv_type, dv_addr, dv_len):
        _req_data = Data()
        _req_data_size = 12
        _req_data.int3dArr[0] = dv_type
        _req_data.int3dArr[1] = dv_addr
        if dv_len > 20:
            print("Length should be less than 20, but {}".format(dv_len))
            return
        _req_data.int3dArr[2] = dv_len

        error_code, _res_data, _res_data_size = self._handle_command(CMD_READ_DIRECT_VARIABLES, _req_data,
                                                                     _req_data_size)

        if not error_code:
            if dv_type == DIRECT_VAR_TYPE_BYTE:  # B
                if _res_data_size == 1 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        res.append(np.array(_res_data.byteArr)[dv_n])
                    return res

            elif dv_type == DIRECT_VAR_TYPE_WORD:  # W
                if _res_data_size == 2 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        val = np.array(_res_data.wordArr)[dv_n].tolist()
                        res.append(int.from_bytes(val, byteorder='little', signed=True))
                    return res

            elif dv_type == DIRECT_VAR_TYPE_DWORD:  # I
                if _res_data_size == 4 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        val = np.array(_res_data.dwordArr)[dv_n].tolist()
                        res.append(int.from_bytes(val, byteorder='little', signed=True))
                    return res

            elif dv_type == DIRECT_VAR_TYPE_LWORD:  # L
                if _res_data_size == 8 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        val = np.array(_res_data.lwordArr)[dv_n].tolist()
                        res.append(int.from_bytes(val, byteorder='little', signed=True))
                    return res

            elif dv_type == DIRECT_VAR_TYPE_FLOAT:  # F
                if _res_data_size == 4 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        res.append(np.array(_res_data.floatArr)[dv_n])
                    return res

            elif dv_type == DIRECT_VAR_TYPE_DFLOAT:  # D
                if _res_data_size == 8 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        res.append(np.array(_res_data.doubleArr)[dv_n])
                    return res

            elif dv_type == DIRECT_VAR_TYPE_MODBUS_REG:  # M
                if _res_data_size == 2 * dv_len:
                    res = []
                    for dv_n in range(0, dv_len):
                        val = np.array(_res_data.uwordArr)[dv_n].tolist()
                        res.append(int.from_bytes(val, byteorder='little', signed=False))
                    return res
            else:
                print("None matched type")
                return False
        else:
            return error_code

    def write_direct_variable(self, dv_type, dv_addr, val):
        _req_data = Data()
        _req_data_size = 8
        _req_data.int2dArr[0] = dv_type
        _req_data.int2dArr[1] = dv_addr

        if dv_type == DIRECT_VAR_TYPE_BYTE:
            memmove(addressof(_req_data.byte) + 8, pointer(c_uint8(val)), 1)
            _req_data_size += 1
        elif dv_type == DIRECT_VAR_TYPE_WORD:
            memmove(addressof(_req_data.byte) + 8, pointer(c_int16(val)), 2)
            _req_data_size += 2
        elif dv_type == DIRECT_VAR_TYPE_DWORD:
            memmove(addressof(_req_data.byte) + 8, pointer(c_int32(val)), 4)
            _req_data_size += 4
        elif dv_type == DIRECT_VAR_TYPE_LWORD:
            memmove(addressof(_req_data.byte) + 8, pointer(c_int64(val)), 8)
            _req_data_size += 8
        elif dv_type == DIRECT_VAR_TYPE_FLOAT:
            memmove(addressof(_req_data.byte) + 8, pointer(c_float(val)), 4)
            _req_data_size += 4
        elif dv_type == DIRECT_VAR_TYPE_DFLOAT:
            memmove(addressof(_req_data.byte) + 8, pointer(c_double(val)), 8)
            _req_data_size += 8
        elif dv_type == DIRECT_VAR_TYPE_MODBUS_REG:
            memmove(addressof(_req_data.byte) + 8, pointer(c_uint16(val)), 2)
            _req_data_size += 2
        else:
            print("None matched type")

        self._handle_command(CMD_WRITE_DIRECT_VARIABLE, _req_data, _req_data_size)

    def write_direct_variables(self, dv_type, dv_addr, dv_len, val):
        _req_data = Data()
        _req_data_size = 12
        _req_data.int3dArr[0] = dv_type
        _req_data.int3dArr[1] = dv_addr
        _req_data.int3dArr[2] = dv_len

        if dv_type == DIRECT_VAR_TYPE_BYTE:
            for ii in range(0, dv_len):
                memmove(addressof(_req_data.byte) + 12 + 1 * ii, pointer(c_uint8(val[ii])), 1)
                _req_data_size += 1

        elif dv_type == DIRECT_VAR_TYPE_WORD:
            for ii in range(0, dv_len):
                type_size = 2
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_int16(val[ii])), type_size)
                _req_data_size += type_size

        elif dv_type == DIRECT_VAR_TYPE_DWORD:
            for ii in range(0, dv_len):
                type_size = 4
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_int32(val[ii])), type_size)
                _req_data_size += type_size

        elif dv_type == DIRECT_VAR_TYPE_LWORD:
            for ii in range(0, dv_len):
                type_size = 8
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_int64(val[ii])), type_size)
                _req_data_size += type_size

        elif dv_type == DIRECT_VAR_TYPE_FLOAT:
            for ii in range(0, dv_len):
                type_size = 4
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_float(val[ii])), type_size)
                _req_data_size += type_size

        elif dv_type == DIRECT_VAR_TYPE_DFLOAT:
            for ii in range(0, dv_len):
                type_size = 8
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_double(val[ii])), type_size)
                _req_data_size += type_size

        elif dv_type == DIRECT_VAR_TYPE_MODBUS_REG:
            for ii in range(0, dv_len):
                type_size = 2
                memmove(addressof(_req_data.byte) + 12 + type_size * ii, pointer(c_uint16(val[ii])), type_size)
                _req_data_size += type_size
        else:
            print("None matched type")

        self._handle_command(CMD_WRITE_DIRECT_VARIABLES, _req_data, _req_data_size)

    @tcp_command(CMD_GET_INV_KIN, 'jointArr')
    def get_inv_kin(self, task_pos, init_q):
        data = Data()
        data_size = (JOINT_DOF + 6) * 8

        for i in range(6):
            data.doubleArr[i] = task_pos[i]

        for i in range(JOINT_DOF):
            data.doubleArr[i + 6] = init_q[i]

        return (data, data_size)

    @tcp_command(CMD_SET_SYNC_MODE)
    def set_sync_mode(self, isSyncMode):
        data = Data()
        data_size = 1
        data.boolVal = isSyncMode
        return (data, data_size)

    ############################################################################
    ## Extended IndyDCP command (Check all)                                    #
    ############################################################################
    def move_ext_traj_bin(self, traj_type, traj_freq, dat_size, traj_data, dat_num=3):
        opt_len = 5
        dat_len = len(traj_data)
        opt_data = [None] * opt_len
        opt_data[0] = traj_type
        opt_data[1] = traj_freq
        opt_data[2] = dat_num
        opt_data[3] = dat_size
        opt_data[4] = int(dat_len / (dat_size * dat_num))  # traj_len

        ext_data1 = np.array(opt_data).tobytes()
        ext_data2 = np.array(traj_data).tobytes()
        req_ext_data = ext_data1 + ext_data2
        req_ext_data_size = len(req_ext_data)

        self._handle_extended_command(EXT_CMD_MOVE_TRAJ_BY_DATA,
                                      req_ext_data,
                                      req_ext_data_size)

    def move_ext_traj_txt(self, traj_type, traj_freq, dat_size, traj_data, dat_num=3):
        opt_len = 5
        dat_len = len(traj_data)
        ext_data_size = opt_len + dat_len
        ext_data = [None] * ext_data_size
        ext_data[0] = traj_type
        ext_data[1] = traj_freq
        ext_data[2] = dat_num
        ext_data[3] = dat_size
        ext_data[4] = int(dat_len / (dat_size * dat_num))  # traj_len
        ext_data[5:-1] = traj_data

        ext_data_str = ' '.join(str(e) for e in ext_data)
        req_ext_data = ext_data_str.encode('ascii')
        req_ext_data_size = len(ext_data_str)

        self._handle_extended_command(EXT_CMD_MOVE_TRAJ_BY_TXT_DATA,
                                      req_ext_data,
                                      req_ext_data_size)

    def move_ext_traj_bin_file(self, file_name):
        file_name += "\0"  # last char should be null
        req_ext_data = file_name.encode('ascii')
        req_ext_data_size = len(file_name)
        self._handle_extended_command(EXT_CMD_MOVE_TRAJ_BY_FILE,
                                      req_ext_data,
                                      req_ext_data_size)

    def move_ext_traj_txt_file(self, file_name):
        file_name += "\0"  # last char should be null
        req_ext_data = file_name.encode('ascii')
        req_ext_data_size = len(file_name)
        self._handle_extended_command(EXT_CMD_MOVE_TRAJ_BY_TXT_FILE,
                                      req_ext_data,
                                      req_ext_data_size)

    def joint_move_to_wp_set(self):
        pass

    def task_move_to_wp_set(self):
        pass

    ############################################################################
    ## JSON program
    ############################################################################
    def set_json_program(self):
        pass

    def set_and_start_json_program(self, json_string):
        json_string += "\0"
        req_ext_data = json_string.encode('ascii')
        req_ext_data_size = len(json_string)
        self._handle_extended_command(EXT_CMD_SET_JSON_PROG_START,
                                      req_ext_data,
                                      req_ext_data_size)

    def wait_for_program_finish(self):
        while self.get_program_state()['running']:
            pass
        print('Program Finished: ', GLOBAL_DICT['stop'])
        return GLOBAL_DICT['stop']

    def wait_for_move_finish(self):
        while self.get_robot_status()['busy']:
            pass
        while self.get_robot_status()['movedone']:
            print("Move finished!")
            return True

    def set_workspace(self, cmd_pos):
        if np.all(cmd_pos != 0):
            return True
        else:
            return False


############################################################################
## Teaching points                                                         #
############################################################################
def load_teaching_data(file_name):
    with open(file_name, "r") as json_file:
        teach_config = json.load(json_file)
        return teach_config


def update_teaching_data(file_name, wp_name, j_pos):
    new_pos = []
    for pos in j_pos:
        new_pos.append(float(pos))

    teach_data = {wp_name: new_pos}

    # If not an exist file
    if not os.path.isfile(file_name):
        with open(file_name, "w+") as f:
            json.dump(teach_data, f)
            return teach_data
    #
    with open(file_name, "r") as json_file:
        teach_config = json.load(json_file)
        teach_config.update(teach_data)

    with open(file_name, "w+") as json_file:
        json.dump(teach_config, json_file)
        return teach_config


def del_teaching_data(file_name, wp_name):
    with open(file_name) as json_file:
        teach_config = json.load(json_file)
        del teach_config[wp_name]

    with open(file_name, 'w') as f:
        json.dump(teach_config, f)

    return teach_config


###############################################################################
# Test                                                                        #
###############################################################################
if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('{0} <Server IP> <Robot Name>'.format(sys.argv[0]))
        sys.exit()

    _server_ip = sys.argv[1]
    _name = sys.argv[2]

    # Connect
    indy = IndyDCPClient(_server_ip, _name)
    indy.connect()

    # Check robot ready
    print('### Test: IsReady() ###')
    if indy.is_robot_ready():
        print('Robot is ready!')
    else:
        print('Robot is not ready!')

    # Check moving finished
    print('### Test: IsMoveFinished() ###')
    if indy.is_move_finished():
        print('Robot is not moving!')
    else:
        print('Robot is moving!')

    # Check DirectTeaching
    print('### Test: StartDirectTeaching() ###')
    if indy.direct_teaching(True):
        print('Start DirectTeaching success!')
    else:
        print('Start DirectTeaching failed!')

    print('### Test: StopDirectTeaching() ###')
    if indy.direct_teaching(False):
        print('Stop DirectTeaching success!')
    else:
        print('Stop DirectTeaching failed!')

    # Get Task Position
    print('### Test: GetTaskPos() ###')
    task_pos = indy.get_task_pos()
    print("Task Pos: ")
    print(task_pos)

    # Get Joint Position
    print('### Test: GetJointPos() ###')
    joint_pos = indy.get_joint_pos()
    print("Joint Pos: ")
    print(joint_pos)

    # Move to Task
    print('### Test: MoveToT() ###')
    indy.task_move_to(task_pos)

    # Move to Joint
    print('### Test: MoveToJ() ###')
    indy.joint_move_to(joint_pos)
    # Disconnect
    indy.disconnect()
    print("Test finished")
