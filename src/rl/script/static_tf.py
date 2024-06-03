#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from utils import *

rospy.init_node('set_static_tf')

# recieve arguments
t = rospy.get_param('~tf').split(' ')
t = list(map(float, t))

header_frame_id = rospy.get_param('~header_frame_id')
child_frame_id = rospy.get_param('~child_frame_id')

tf = TF()
tf.pub_static_tf_orientation(t, header_frame_id, child_frame_id)

rospy.spin()