#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import CameraInfo
import numpy as np

# _T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.57, -1.35, 0.0])
# _T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.47, -1.45, 0.2])
# _T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.605, -1.36, -0.035])
# _T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.47, -1.45, 0.2])
# _T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.37, -1.55, 0.2])
_T_cam_velo = np.array([0.0, -0.0583, -0.015, 1.6, -1.36, -0.035])

# _T_cam_velo = np.array([10.0, 0.0, 0, 0, 0, 0.0])
# _T_cam_velo = np.array([-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057])
fx = 1358.010977
fy = 1357.44302436
cx = 950.243
cy = 644.85837
image_w = 1920; image_h = 1200

if __name__ == '__main__':
    rospy.init_node('tf_cameraInfo_broadcaster')
    br = tf.TransformBroadcaster()
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        br.sendTransform(_T_cam_velo[0:3].flatten().tolist(),
            tf.transformations.quaternion_from_euler(*_T_cam_velo[3:6].flatten().tolist()),
            rospy.Time.now(),
            "velodyne",
            "camera")
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera"
        camera_info.height = image_h
        camera_info.width = image_w
        camera_info.distortion_model = 'plumb_bob'
        camera_info.D = [-0.21513368608758962, 0.1462154723060369, 0.0005687877808524363, 0.0011074967324128063, 0.0]
        camera_info.K = [fx,0,cx,0,fy,cy,0,0,1]
        camera_info.P = [fx,0,cx,0,0,fy,cy,0,0,0,1,0]
        camera_info_pub.publish(camera_info)
        rate.sleep()
