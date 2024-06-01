#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from utils.utils import *

rospy.init_node('aruco_pose_estimation')

# recieve arguments
test = rospy.get_param('~test').split(' ')

# list string to list int
test = list(map(int, test))

rospy.loginfo(f"args: {test}")

tf = TF()

tf.pub_tf_static(tf=test, header_frame_id="world", child_frame_id="aruco_locate")

rospy.spin()