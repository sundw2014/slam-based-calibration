#!/usr/bin/env python
import rospy
import tf
import numpy as np
import sys

if __name__ == '__main__':
    rospy.init_node('calibration')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    ORB_trajectory = {'trans': list(), 'rot': list()}
    lidar_trajectory = {'trans': list(), 'rot': list()}

    while not rospy.is_shutdown():
        try:
            (ORB_camera_trans, ORB_camera_rot) = listener.lookupTransform('ORB_SLAM/Camera', 'ORB_SLAM/World', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            (lidar_trans, lidar_rot) = listener.lookupTransform('laser_odom', 'camera_init', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        ORB_trajectory['trans'].append(ORB_camera_trans)
        ORB_trajectory['rot'].append(ORB_camera_rot)
        lidar_trajectory['trans'].append(lidar_trans)
        lidar_trajectory['rot'].append(lidar_rot)

        print(len(ORB_trajectory['trans']))

        if len(ORB_trajectory['trans']) > 10:
            break;
        rate.sleep()

        ORB_trajectory_trans = np.array(ORB_trajectory['trans'])
        ORB_trajectory_rot = np.array(ORB_trajectory['rot'])
        lidar_trajectory_trans = np.array(lidar_trajectory['trans'])
        lidar_trajectory_rot = np.array(lidar_trajectory['rot'])

        np.savez(sys.argv[1], ORB_trajectory_trans, ORB_trajectory_rot, lidar_trajectory_trans, lidar_trajectory_rot)
