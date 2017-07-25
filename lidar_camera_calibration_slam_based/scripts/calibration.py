#!/usr/bin/env python
import rospy
import tf
import numpy as np
import sys
from tf.transformations import *
import scipy.io as sio

if __name__ == '__main__':
    rospy.init_node('calibration')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    ORB_trajectory_trans = list()
    ORB_trajectory_rot = list()
    lidar_trajectory_trans = list()
    lidar_trajectory_rot = list()

    while not rospy.is_shutdown():
        try:
            (ORB_camera_trans, ORB_camera_rot) = listener.lookupTransform('ORB_SLAM/Camera', 'ORB_SLAM/World', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            (lidar_trans, lidar_rot) = listener.lookupTransform('camera', 'camera_init', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        ORB_trajectory_trans.append(ORB_camera_trans)
        ORB_trajectory_rot.append(ORB_camera_rot)
        lidar_trajectory_trans.append(lidar_trans)
        lidar_trajectory_rot.append(lidar_rot)

        print(len(ORB_trajectory_trans))

        if len(ORB_trajectory_trans) > 5000:
            break;
        rate.sleep()

    ORB_trajectory_trans = np.array(ORB_trajectory_trans)
    ORB_trajectory_rot  = np.array(ORB_trajectory_rot)
    lidar_trajectory_trans = np.array(lidar_trajectory_trans)
    lidar_trajectory_rot = np.array(lidar_trajectory_rot)
    np.savez(sys.argv[1],\
        ORB_trajectory_trans=ORB_trajectory_trans,\
        ORB_trajectory_rot=ORB_trajectory_rot,\
        lidar_trajectory_trans=lidar_trajectory_trans,\
        lidar_trajectory_rot=lidar_trajectory_rot)

    Cs = list()
    Ls = list()
    for i in range(ORB_trajectory_trans.shape[0]):
        # Ct means camera translation, Cr means camera rotation
        Ct = ORB_trajectory_trans[i,:]
        Cr = ORB_trajectory_rot[i,:]
        Lt = lidar_trajectory_trans[i,:]
        Lr = lidar_trajectory_rot[i,:]
        # get the homogeneous transform matrix
        Cr = quaternion_matrix(Cr[[3,0,1,2]])
        Ct = translation_matrix(Ct)
        C = np.dot(Ct, Cr)
        Lr = quaternion_matrix(Lr[[3,0,1,2]])
        Lt = translation_matrix(Lt)
        L = np.dot(Lt, Lr)

        Cs.append(C)
        Ls.append(L)

    # transpose to 4x4xM
    Cs = np.array(Cs).transpose((1,2,0))
    Ls = np.array(Ls).transpose((1,2,0))
    sio.savemat(sys.argv[2],{'C_pose':Cs, 'L_pose':Ls})
