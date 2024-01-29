'''
Created on 2019. 3. 15.

@author: JSK
'''
import numpy as np
from math import *
import scipy
from scipy.spatial.transform import Rotation
if not hasattr(Rotation, 'as_dcm'):
    class Rotation(Rotation):
        def as_dcm(self, *args, **kwargs):
            return self.as_matrix(*args, **kwargs)

        @classmethod
        def from_dcm(cls, *args, **kwargs):
            return cls.from_matrix(*args, **kwargs)

##
# @brief calculate rotation vector to rotate vec1 to vec2
# @return rvec: vec2 = rmat(rvec)*vec1
def calc_rotvec_vecs(vec1, vec2):
    cross_vec = np.cross(vec1, vec2)
    dot_val = np.dot(vec1, vec2)
    cross_abs = np.linalg.norm(cross_vec)
    if np.linalg.norm(cross_abs) < 1e-8:
        if len(vec1)==2:
            if dot_val>=0:
                rotvec = 0
            else:
                rotvec = np.pi
        elif len(vec1)==3:
            rotvec = np.zeros(3)
            if dot_val<0:
                rotvec[2] = np.pi
    else:
        cross_nm = cross_vec/cross_abs
        rotvec = cross_nm * np.arctan2(cross_abs, dot_val)
    return rotvec

def rad2deg(rads):
    return np.asarray(rads)/np.pi*180
        
def deg2rad(degs):
    return np.asarray(degs)/180*np.pi

def Rot_axis( axis, q ):
    '''
    make rotation matrix along axis
    '''
    if axis==1:
        R = np.asarray([[1,0,0],[0,cos(q),-sin(q)],[0,sin(q),cos(q)]])
    if axis==2:
        R = np.asarray([[cos(q),0,sin(q)],[0,1,0],[-sin(q),0,cos(q)]])
    if axis==3:
        R = np.asarray([[cos(q),-sin(q),0],[sin(q),cos(q),0],[0,0,1]])
    return R

def Rot_zyx(zr,yr,xr):
    '''
    zyx rotatio matrix - caution: axis order: z,y,x
    '''
    R = np.matmul(np.matmul(Rot_axis(3,zr),Rot_axis(2,yr)),Rot_axis(1,xr))
    return R

def Rot_quat(x,y,z,w):
    return Rotation.from_quat([x,y,z,w]).as_dcm()

def Rot_rpy(roll,pitch,yaw):
    return Rot_zyx(yaw, pitch, roll)

def Rot_zxz(zr1,xr2,zr3):
    '''
    zxz rotatio matrix - caution: axis order: z,x,z
    '''
    R = np.matmul(np.matmul(Rot_axis(3,zr1),Rot_axis(1,xr2)),Rot_axis(3,zr3))
    return R

def Rot2zyx(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    R = np.asarray(R)
    sy = sqrt(R[0,0]**2 + R[1,0]**2)

    if sy > 0.000001:
        x = atan2(R[2,1] , R[2,2])
        y = atan2(-R[2,0], sy)
        z = atan2(R[1,0], R[0,0])
    else:
        x = atan2(-R[1,2], R[1,1])
        y = atan2(-R[2,0], sy)
        z = 0
    return np.asarray([z,y,x])

def Rot2quat(R):
    R = np.asarray(R)
    return Rotation.from_dcm(R).as_quat()

def Rot2rpy(R):
    R = np.asarray(R)
    return np.asarray(list(reversed(Rot2zyx(R))))

def Rot2zxz(R):
    '''
    rotatio matrix to zyx angles - caution: axis order: z,y,x
    '''
    sy = sqrt(R[0,2]**2 + R[1,2]**2)

    if sy > 0.000001:
        z1 = atan2(R[0,2] , -R[1,2])
        x2 = atan2(sy,R[2,2])
        z3 = atan2(R[2,0], R[2,1])
    else:
        z1 = 0
        x2 = atan2(sy,R[2,2])
        z3 = atan2(-R[0,1], R[0,0])
    return np.asarray([z1,x2,z3])

def quat2yaw(quat):
    return Rotation.from_quat(quat).as_rotvec()[2]

def yaw2quat(yaw):
    return Rotation.from_rotvec([0,0,yaw]).as_quat()

def mod_peri_zero(x, peri):
    return (x + peri / 2) % peri - peri / 2

def SE3(R,P):
    T = np.identity(4,dtype='float32')
    T[0:3,0:3]=R
    T[0:3,3]=P
    return T
    
def SE3_inv(T):
    R=T[0:3,0:3].transpose()
    P=-np.matmul(R,T[0:3,3])
    return (SE3(R,P))
    
def SE3_R(T):
    return T[0:3,0:3]
    
def SE3_P(T):
    return T[0:3,3]
   
def SE3_mul_vec3(T,v):
    r=np.matmul(SE3_R(T),v)
    return np.add(r,SE3_P(T))

def average_SE3(Ts):
    nT = Ts.shape[0]
    Rref = Ts[0,:3,:3]
    dRlie_list = []
    for i in range(nT):
        Ri = Ts[i,:3,:3]
        dRi = np.matmul(Rref.transpose(),Ri)
        dRlie = np.real(scipy.linalg.logm(dRi,disp=False)[0])
        dRlie_list += [dRlie]
    dRlie_list = np.array(dRlie_list)
    dRlie_m = np.mean(dRlie_list,axis=0)
    R_m = np.matmul(Rref,scipy.linalg.expm(dRlie_m))
    P_m = np.mean(Ts[:,:3,3],axis=0)
    T_m=SE3(R_m,P_m)
    return T_m

def T2cmd(Tbe):
    Tbe_cmd = np.zeros([6])
    Tbe_cmd[0:3] = Tbe[0:3, 3]
    Tbe_cmd[3:6] = np.rad2deg(Rot2zyx(Tbe[0:3, 0:3]))[[2, 1, 0]]
    return Tbe_cmd

##
# @param Tbe_cmd  m, degs
def cmd2T(Tbe_cmd):
    Pbe = Tbe_cmd[0:3]
    rotbe = np.deg2rad(Tbe_cmd[3:6])
    Rbe = Rot_zyx(rotbe[2], rotbe[1], rotbe[0])
    Tbe = SE3(Rbe, Pbe)
    return Tbe

Tx180 = np.identity(4, 'float32')
Tx180[1,1]=-1
Tx180[2,2]=-1

Ty180 = np.identity(4, 'float32')
Ty180[0,0]=-1
Ty180[2,2]=-1