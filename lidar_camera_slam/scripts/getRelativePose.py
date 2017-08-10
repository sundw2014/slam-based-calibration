import sys
import pykitti
import argparse
import numpy as np
import transformation

parser = argparse.ArgumentParser()
parser.add_argument('--basedir', default='.', help='kitti basedir [default: .]')
parser.add_argument('--date', default='2011_09_30', help='kitti date [default: 2011_09_30]')
parser.add_argument('--drive', default='0028', help='kitti drive [default: 0028]')
parser.add_argument('--frame1', default=1, type=int, help='frame1 ID [default: 1]')
parser.add_argument('--frame2', default=2, type=int, help='frame2 ID [default: 2]')

FLAGS = parser.parse_args()

basedir = FLAGS.basedir
date = FLAGS.date
drive = FLAGS.drive
frames = [FLAGS.frame1, FLAGS.frame2]
kitti = pykitti.raw(basedir, date, drive, frames=frames)

datas = zip(kitti.velo, kitti.oxts)
velo_raw_1, oxts_1 = datas[0]
velo_raw_2, oxts_2 = datas[1]

T_w_velo1 = oxts_1.T_w_imu.dot(np.linalg.inv(kitti.calib.T_velo_imu))
T_w_velo2 = oxts_2.T_w_imu.dot(np.linalg.inv(kitti.calib.T_velo_imu))

relative_pose = np.linalg.inv(T_w_velo2).dot(T_w_velo1) # T_fixed_moving

ax, ay, az = transformation.euler_from_matrix(relative_pose[0:3,0:3], axes='sxyz')
pose = np.zeros((6))
pose[0:3] = relative_pose[0:3,3].flatten()
pose[3:6] = [ax, ay, az]
print(pose)
