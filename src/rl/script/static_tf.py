#!/usr/bin/env python3
# 從rl.launch 獲取參數 發佈 static_tf 
from scipy.spatial.transform import Rotation as R
import rospy
from utils import *

rospy.init_node('set_static_tf')

# 接收參數
t = rospy.get_param('~tf').split(' ')
# 轉換為浮點數list
t = list(map(float, t))

header_frame_id = rospy.get_param('~header_frame_id')
child_frame_id = rospy.get_param('~child_frame_id')

tf = TF()
tf.pub_static_tf_orientation(t, header_frame_id, child_frame_id)
# 保持節點運行
rospy.spin()