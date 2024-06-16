#!/usr/bin/env python3

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
index = rospy.get_param(f'~/capture{camera}/index')
camera_matrix = rospy.get_param(f'~/capture{camera}/camera_matrix_file')
dist_coeffs = rospy.get_param(f'~/capture{camera}/distortion_coefficients_file')

# rospy.loginfo('locate_camera: %s, %s', camera_matrix, dist_coeffs)

tf = TF()

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

aruco_params = cv2.aruco.DetectorParameters()

DISPLAY = rospy.get_param('~display')
MARKER_SIZE = 0.07  

camera_matrix = np.load(camera_matrix)
dist_coeffs = np.load(dist_coeffs)

cv_bridge = CvBridge()

while running:
    frame = rospy.wait_for_message(f'camera{camera}', Image)
    
    frame = cv_bridge.imgmsg_to_cv2(frame, 'passthrough').copy()

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

            quat = quaternion_conjugate(quat)

            t = (camera_position[0], camera_position[1], camera_position[2], quat[0], quat[1], quat[2], quat[3])

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
