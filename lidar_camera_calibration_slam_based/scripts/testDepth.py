#!/usr/bin/env python
import rospy
from lidar_camera_calibration_slam_based.msg import keyframeMsg
import cv2
import pykitti
import numpy as np
import struct

def cv_imshow(im):
    cv2.namedWindow('window', cv2.WINDOW_NORMAL)
    cv2.imshow('window', im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def callback(data):
    from IPython import embed; embed()

def unpack_generator(buffer):
    for offset in range(0, len(buffer)-1, 12):
        yield struct.unpack_from('<ffBBBB', buffer, offset=offset)

def get_depthImage(msg):
    depth = np.zeros((msg.height, msg.width))
    points = unpack_generator(msg.pointcloud)
    for i in range(msg.height):
        for j in range(msg.width):
            point = points.next()
            depth[i,j] = 1.0 / point[0]
    return depth

def get_grayscaleImage(msg):
    gray = np.zeros((msg.height, msg.width))
    points = unpack_generator(msg.pointcloud)
    for i in range(msg.height):
        for j in range(msg.width):
            point = points.next()
            gray[i,j] = point[2]
    return gray

def get_lidar_depthImage(msg, kitti):
    depth = -1.0 * np.ones((msg.height, msg.width))
    T_cam3_velo = kitti.calib.T_cam3_velo
    K = np.array([[msg.fx,0,msg.cx],[0,msg.fy,msg.cy],[0,0,1]])
    velo = kitti.velo.next()
    velo[:,3] = 1.0
    camera_point = K.dot(T_cam3_velo.dot(velo.transpose())[0:3,:])
    z = camera_point[2,:]
    u = camera_point[0,:] / z
    v = camera_point[1,:] / z
    idx = np.where((u>0) & (u<msg.width) & (v>0) & (v<msg.height) & (z>0))[0]
    depth[v[idx].astype('int'), u[idx].astype('int')] = z[idx]
    return depth

idx1, idx2 = np.where((d1>0) & (d2>0))
X = d1[idx1, idx2]
Y = d2[idx1, idx2]

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/lsd_slam/keyframes", keyframeMsg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
