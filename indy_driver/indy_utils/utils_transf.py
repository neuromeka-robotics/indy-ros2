import math
import numpy as np
# import tf
# from geometry_msgs.msg import Pose

def rads2degs(rad_list):
    degs = [math.degrees(rad) for rad in rad_list]
    return degs

def degs2rads(deg_list):
    rads = [math.radians(deg) for deg in deg_list]
    return rads

# def xyz_uvw_from_pose(p):
#     xyz_uvw = [0, 0, 0, 0, 0, 0]
#     xyz_uvw[0] = p.position.x
#     xyz_uvw[1] = p.position.y
#     xyz_uvw[2] = p.position.z

#     q = np.array((p.orientation.x,
#                         p.orientation.y,
#                         p.orientation.z,
#                         p.orientation.w))

#     uvw = tf.transformations.euler_from_quaternion(q)

#     xyz_uvw[3] = uvw[0]
#     xyz_uvw[4] = uvw[1]
#     xyz_uvw[5] = uvw[2]

#     return xyz_uvw

# def pose_from_xyz_uvw(xyz_uvw):

#     p = Pose()
#     p.position.x = xyz_uvw[0]
#     p.position.y = xyz_uvw[1]
#     p.position.z =  xyz_uvw[2]

#     q = tf.transformations.quaternion_from_euler(xyz_uvw[3], xyz_uvw[4], xyz_uvw[5])

#     p.orientation.x = q[0]
#     p.orientation.y = q[1]
#     p.orientation.z = q[2]
#     p.orientation.w = q[3]

#     return p