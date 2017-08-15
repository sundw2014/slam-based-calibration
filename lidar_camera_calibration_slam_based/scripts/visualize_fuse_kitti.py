#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import time
import numpy as np
import transformation

image = None
pc = None

class image_converter:

  def __init__(self, image_topic):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # print("%0.9f" % (data.header.stamp.secs + data.header.stamp.nsecs / 1e9))
    global image
    image = cv_image
    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

def pc_callback(data):
    print('new pointcloud')
    global pc
    pc = np.frombuffer(data.data, dtype=np.float32, count=data.row_step/4)

_T_cam_velo = np.array([-0.00478403, -0.07337429, -0.33399681, -2.870498845517217, -1.5640538287333043, -1.849936230550016])
# _T_cam_velo = np.array([-0.47637765, -0.07337429, -0.33399681, -2.8704988456, -1.56405382877, -1.84993623057])
fx = 707.0912
fy = 707.0912
cx = 601.8873
cy = 183.1104
camera_K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
image_w = 1226; image_h = 370
T_cam_velo = transformation.compose_matrix(angles = _T_cam_velo[3:6], translate = _T_cam_velo[0:3])

def transform_show(velo, local_image):
    velo = velo.copy()
    velo = velo.reshape((-1, 4)).transpose()
    velo[3,:] = 1
    # print(velo[:,0:5])
    image_points = camera_K.dot(T_cam_velo[0:3, :]).dot(velo)
    local_image = local_image.copy()
    for i in range(image_points.shape[1]):
        image_point = image_points[:, i]
        u = image_point[0]
        v = image_point[1]
        z = image_point[2]
        u = int(u/z); v = int(v/z);
        if u<0 or u>=image_w or v<0 or v>=image_h or z<0:
            continue
        cv2.circle(local_image, (u,v), 1, 255)
    cv2.imshow('fuse', local_image) #cv2.resize(local_image, (0, 0), fx = 0.5, fy = 0.5))
    cv2.waitKey(30)

def main(args):
  ic = image_converter(args[1])
  pc_sub = rospy.Subscriber(args[2],PointCloud2,pc_callback)
  rospy.init_node('visualize_fuse', anonymous=True)
  try:
    while True:
        global image, pc
        if image is not None and pc is not None:
            transform_show(pc, image)
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
