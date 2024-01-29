import sys
import json
import grpc
import math
# from bitarray import bitarray
# from bitarray.util import int2ba

sys.path.append("../gRPCServerGenPython")
sys.path.append("../utils")

from EtherCATCommgRPCServer_pb2_grpc import GRPCECatTaskStub
from EtherCATCommgRPCServer_pb2 import *



class GRPCECatTask:
    def __init__(self, step_ip='127.0.0.1'):
        # initialize RPC
        
        self.channel = grpc.insecure_channel('%s:50050'%step_ip)
        self.stub = GRPCECatTaskStub(self.channel)

    
    ###### EtherCAT Communication Task gRPC protocol
    def get_master_status(self):
        status = self.stub.GetMasterStatus(Empty()).val
        if status == 1:
            return "INIT"
        elif status == 2:
            return "PRE-OP"
        elif status == 4:
            return "SAFE-OP"
        elif status == 8:
            return "OP"
        else:
            return "None"
        
    def get_slave_status(self):
        status = list(self.stub.GetSlaveStatus(Empty()).val)        
        slave_status = []        
        for stat in status:
            if stat == 1:
                slave_status.append("INIT")                
            elif stat == 2:
                slave_status.append("PRE-OP")                
            elif stat == 4:
                slave_status.append("SAFE-OP")                
            elif stat == 8:
                slave_status.append("OP")                
            else:
                slave_status.append("None")
        
        return slave_status
        
    
    def get_rxdomain_status(self):
        status = self.stub.GetRxDomainStatus(Empty()).val
        if status == 0:
            return "ZERO"
        elif status == 1:
            return "INCOMPLETE"
        elif status == 2:
            return "COMPLETE"        
        else:
            return "None"       
    
    def get_txdomain_status(self):
        status = self.stub.GetTxDomainStatus(Empty()).val
        if status == 0:
            return "ZERO"
        elif status == 1:
            return "INCOMPLETE"
        elif status == 2:
            return "COMPLETE"        
        else:
            return "None"   
        
    def is_system_ready(self):
        return list(self.stub.IsSystemReady(Empty()).val)
    
    def set_servo(self, slave_idx, state):
        return self.stub.SetServoOnOff(ServoIndex(ecatIndex=slave_idx, servoState=state))
        
    ## PDO processing (motor driver)
    def set_md_rx_pdo(self, slave_idx, controlWord, modeOp, targetPos, targetVel, targetTor):
        mdRx = MotorDriverRx(controlWord=controlWord, modeOp=modeOp, targetPosition=targetPos, targetVelocity=targetVel, targetTorque=targetTor)
        return self.stub.SetRxPDOMotorDriver(MotorDriverRxIndex(slaveIdx=slave_idx, motorDriverRx=mdRx))
    
    def get_md_rx_pdo(self, slave_idx):
        mdRx = self.stub.GetRxPDOMotorDriver(IntVal(val=slave_idx))
        return [mdRx.controlWord, mdRx.modeOp, mdRx.targetPosition, mdRx.targetVelocity, mdRx.targetTorque]
    
    def get_md_tx_pdo(self, slave_idx):
        mdTx = self.stub.GetTxPDOMotorDriver(IntVal(val=slave_idx))
        return [mdTx.statusWord, mdTx.modeOpDisp, mdTx.actualPosition, mdTx.actualVelocity, mdTx.actualTorque]

    def set_panasonic_rx_pdo(self, slave_idx, controlWord, modeOp, targetPos, targetVel, targetTor, maxTor, maxSpd, touch):
        mdRx = PanasonicDriverRx(controlWord=controlWord, modeOp=modeOp, targetPosition=targetPos, targetVelocity=targetVel, targetTorque=targetTor, maxTorque=maxTor, maxMotorSpeed=maxSpd, touchProbe=touch)
        return self.stub.SetRxPDOPanasonicDriver(PanasonicDriverRxIndex(slaveIdx=slave_idx, motorDriverRx=mdRx))    
    
    def get_panasonic_tx_pdo(self, slave_idx):
        mdTx = self.stub.GetTxPDOPanasonicDriver(IntVal(val=slave_idx))
        return [mdTx.statusWord, mdTx.modeOpDisp, mdTx.actualPosition, mdTx.actualVelocity, mdTx.actualTorque, mdTx.errorCode]
    
    ## PDO processing (IO board)
    def get_ioboard_do(self):
        ioRx = self.stub.GetNRMKIOBoardOutput(Empty())       
        do1 = bin(ioRx.do1)[2:].zfill(8)
        do2 = bin(ioRx.do2)[2:].zfill(8)
        do5v = bin(ioRx.do_5v)[2:].zfill(4)

        do1 = list(map(int, do1))
        do2 = list(map(int, do2))
        do5v = list(map(int, do5v))

        do_list = do1 + do2 + do5v
        
        return do_list
    
    def get_ioboard_di(self):
        ioTx = self.stub.GetNRMKIOBoardInput(Empty())       
        di1 = bin(ioTx.di1)[2:].zfill(8)
        di2 = bin(ioTx.di2)[2:].zfill(8)
        di5v = bin(ioTx.di_5v)[2:].zfill(4)

        di1 = list(map(int, di1))
        di2 = list(map(int, di2))
        di5v = list(map(int, di5v))

        di_list = di1 + di2 + di5v
        
        return di_list
    
    def set_ioboard_do(self, do_idx, do_val):
        do_list = self.get_ioboard_do()        
        do_list[do_idx] = do_val
        do1 = do_list[0:8]
        do2 = do_list[8:16]
        do5v = do_list[16:20]

        do1_str = [str(x) for x in do1]
        do2_str = [str(x) for x in do2]
        do5v_str = [str(x) for x in do5v]
        do1_str = ''.join(do1_str)
        do2_str = ''.join(do2_str)
        do5v_str = ''.join(do5v_str)

        ioRx = IOBoardRx(do_5v=int(do5v_str,2), do1=int(do1_str,2), do2=int(do2_str,2), ao1=0, ao2=0, ft_param=0)
        return self.stub.SetNRMKIOBoardOutput(ioRx)
    
    def set_ioboard_dos(self, do_vals):
        do_list = do_vals      
        do1 = do_list[0:8]
        do2 = do_list[8:16]
        do5v = do_list[16:20]

        do1_str = [str(x) for x in do1]
        do2_str = [str(x) for x in do2]
        do5v_str = [str(x) for x in do5v]
        do1_str = ''.join(do1_str)
        do2_str = ''.join(do2_str)
        do5v_str = ''.join(do5v_str)

        ioRx = IOBoardRx(do_5v=int(do5v_str,2), do1=int(do1_str,2), do2=int(do2_str,2), ao1=0, ao2=0, ft_param=0)
        return self.stub.SetNRMKIOBoardOutput(ioRx)
    
    def led_g(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def led_b(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def led_r(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
    def led_gb(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    def led_rg(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    def led_rb(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        
    def led_rgb(self):
        self.set_ioboard_dos([0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    
    
    ## Reset Welcon driver
    def get_ft_sensor(self, slave_idx):
        data = self.stub.GetRobotusFTSensor(IntVal(val=slave_idx))
        return [data.fx, data.fy, data.fz, data.tx, data.ty, data.tz]
    
    ## Reset Welcon driver
    def reset_welcon(self, slave_idx):
        return self.stub.ResetWelconDriver(IntVal(val=slave_idx))

    ## Get SDO
    def get_error_code(self, slave_idx):
        return self.stub.GetErrorCode(IntVal(val=slave_idx)).val
    
    def get_maxTorque(self, slave_idx):
        return self.stub.GetMaxTorque(IntVal(val=slave_idx)).val
    
    def get_profileVel(self, slave_idx):
        return self.stub.GetProfileVelocity(IntVal(val=slave_idx)).val
    
    def get_profileAcc(self, slave_idx):
        return self.stub.GetProfileAcc(IntVal(val=slave_idx)).val
    
    def get_profileDec(self, slave_idx):
        return self.stub.GetProfileDec(IntVal(val=slave_idx)).val
    
    ## Set SDO
    def set_maxTorque(self, slave_idx, value):
        return self.stub.SetMaxTorque(IntVals(val=[slave_idx, value]))
    
    def set_profileVel(self, slave_idx, value):
        return self.stub.SetProfileVelocity(IntVals(val=[slave_idx, value]))
    
    def set_profileAcc(self, slave_idx, value):
        return self.stub.SGetProfileAcc(IntVals(val=[slave_idx, value]))
    
    def set_profileDec(self, slave_idx, value):
        return self.stub.SetProfileDec(IntVals(val=[slave_idx, value]))
    