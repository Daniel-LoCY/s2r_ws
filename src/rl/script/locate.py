#!/usr/bin/env python3
# 兩個camera <--> 兩個aruco TF

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from utils import *
import signal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tf.transformations import quaternion_conjugate

running = True

def signal_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, signal_handler)
rospy.init_node(f'locate_camera')

camera = rospy.get_param('~camera')
# 相機index
index = rospy.get_param(f'~/capture{camera}/index')
# 相機矩陣文件路徑
camera_matrix = rospy.get_param(f'~/capture{camera}/camera_matrix_file')
# 畸變係數文件路徑
dist_coeffs = rospy.get_param(f'~/capture{camera}/distortion_coefficients_file')

# rospy.loginfo('locate_camera: %s, %s', camera_matrix, dist_coeffs)

tf = TF()
# 獲取預定義的 ArUco 字典
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
# 創建 ArUco 檢測參數
aruco_params = cv2.aruco.DetectorParameters()

DISPLAY = rospy.get_param('~display')
MARKER_SIZE = 0.07  

camera_matrix = np.load(camera_matrix)
dist_coeffs = np.load(dist_coeffs)

cv_bridge = CvBridge()

while running:
    frame = rospy.wait_for_message(f'camera{camera}', Image)
    
    frame = cv_bridge.imgmsg_to_cv2(frame, 'passthrough').copy()
    # 角點座標, 標記的 ID, 未被確定為有效 ArUco 標記的候選區域
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            id = ids[i][0]
            # 標記相對於相機的旋轉、平移
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            # 計算相機的位置和方向-----------------------------
            R_mat, _ = cv2.Rodrigues(rvec)
            camera_position = -R_mat.T @ tvec

            rotation = R.from_matrix(R_mat)
            quat = rotation.as_quat()

            quat = quaternion_conjugate(quat)

            t = (camera_position[0], camera_position[1], camera_position[2], quat[0], quat[1], quat[2], quat[3])
            # -----------------------------------------------
            # 發佈 TF 變換
            if id == 1:
                tf.pub_tf_orientation(t, f'aruco_locate_r', f'camera_link{index}')
            elif id == 3:
                tf.pub_tf_orientation(t, f'aruco_locate_l', f'camera_link{index}')


    if DISPLAY:
        cv2.imshow(f'camera{camera}', frame)

        if cv2.waitKey(1) == ord('q'):
            break

if DISPLAY:
    cv2.destroyAllWindows()
