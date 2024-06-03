#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from utils import *

rospy.init_node('aruco_pose_estimation')

tf = TF()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

aruco_params = cv2.aruco.DetectorParameters()

DISPLAY = True
MARKER_SIZE = 0.08  
CAMERA_INDEX = 2
CAMERA_MODEL = 'UCAM-G1'

camera_matrix = np.load(f'camera_calibration/{CAMERA_MODEL}/cam_matrix.npy')
dist_coeffs = np.load(f'camera_calibration/{CAMERA_MODEL}/cam_distortion.npy')

cap = cv2.VideoCapture(CAMERA_INDEX)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    if ids is not None:
        frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            id = ids[i][0]
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            R_mat, _ = cv2.Rodrigues(rvec)
            camera_position = -R_mat.T @ tvec

            rotation = R.from_matrix(R_mat)
            quat = rotation.as_quat()

            # euler_angles = rotation.as_euler('xyz', degrees=True)

            t = (camera_position[0], camera_position[1], camera_position[2], quat[0], quat[1], quat[2], quat[3])

            if id == 0:
                tf.pub_static_tf_orientation(t, f'aruco_locate', 'camera_link')
            else:
                tf.pub_static_tf_orientation(t, f'camera_link', f'object_{id}')

            # print(f"ArUco ID: {id}, 相机在标记中的位置: {camera_position}, 四元数: {quat}, 欧拉角: {euler_angles}")

    if DISPLAY:
        cv2.imshow('color', frame)

        if cv2.waitKey(1) == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
