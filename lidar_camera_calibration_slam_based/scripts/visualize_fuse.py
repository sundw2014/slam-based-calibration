#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import transformation

image = None

class image_converter:

  def __init__(self, image_topic):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    print("%0.9f" % (data.header.stamp.secs + data.header.stamp.nsecs / 1e9))
    global image
    image = cv_image
    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

_T_cam_velo = np.array([0.0, -0.0583, -0.015, -1.57, 0, -1.57])
fx = 1358.010977
fy = 1357.44302436
cx = 950.243
cy = 644.85837
camera_K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
image_w = 1920; image_h = 1200

def transform_show(_T_cam_velo, local_image):
    T_cam_velo = transformation.compose_matrix(angles = _T_cam_velo[3:6], translate = _T_cam_velo[0:3])
    filename = '/home/sundw/data/velodyne_points/770.bin'
    velo = np.fromfile(filename, dtype=np.float32)
    velo = velo.reshape((-1, 4)).transpose()
    transformed_pl = T_cam_velo.dot(velo)
    image_points = camera_K.dot(transformed_pl[0:3, :])
    local_image = local_image.copy()
    for i in range(image_points.shape[1]):
        image_point = image_points[:, i]
        u = image_point[0]
        v = image_point[1]
        z = image_point[2]
        u = int(u/z); v = int(v/z);
        if u<0 or u>=image_w or v<0 or v>=image_h or z<0:
            continue
        cv2.circle(local_image, (v,u), 3, 255)
    cv2.imshow('fuse', cv2.resize(local_image, (0, 0), fx = 0.5, fy = 0.5))
    cv2.waitKey(30)

def main(args):
  ic = image_converter(args[1])
  rospy.init_node('visualize_fuse', anonymous=True)
  try:
    while True:
        global image
        if image is not None:
            from IPython import embed; embed()
            filename = '/home/sundw/data/velodyne_points/770.bin'
            velo = np.fromfile(filename, dtype=np.float32)
            velo = velo.reshape((-1, 4)).transpose()
            image_w = 1920; image_h = 1200
            transformed_pl = T_cam_velo.dot(velo)
            image_points = camera_K.dot(transformed_pl[0:3, :])
            local_image = image.copy()
            for i in range(image_points.shape[1]):
                image_point = image_points[:, i]
                u = image_point[0]
                v = image_point[1]
                z = image_point[2]
                u = int(u/z); v = int(v/z);
                if u<0 or u>=image_w or v<0 or v>=image_h or z<0:
                    continue
                cv2.circle(local_image, (v,u), 3, 255)
            cv2.imshow('fuse', cv2.resize(local_image, (0, 0), fx = 0.5, fy = 0.5))
            cv2.waitKey(100)
            break;
        time.sleep(1)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
