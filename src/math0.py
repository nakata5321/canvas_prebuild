#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros as tf2
import geometry_msgs.msg
import numpy as np

#trans = geometry_msgs.msg.TransformStamped()

def rpy_and_quat_from_xyz(xyz_arr):
    x = xyz_arr[:,0]
    y = xyz_arr[:,1]
    z = xyz_arr[:,2]
    xs = sum(x)/4
    ys = sum(y)/4
    zs = sum(z)/4
    M = np.zeros((4,3))
    for i in range(len(x)):
        xx = x[i] - xs
        yy = y[i] - ys
        zz = z[i] - zs
        M[i] = np.array([xx, yy, zz])
    A = np.dot(M.T, M)
    s, v, d = np.linalg.svd(A)
    eul = tf.transformations.euler_from_matrix(d)
    quat =tf.transformations.quaternion_from_euler(eul[0], eul[1], eul[2])
    return(eul, quat)
