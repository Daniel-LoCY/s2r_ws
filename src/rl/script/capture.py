#!/usr/bin/env python3
# 相機 pub資料 到 tf程式(loacte、obj_pose_estimation)

import cv2
import rospy
# ROS 中用於影像傳遞的消息類型
from sensor_msgs.msg import Image
# ROS 提供的 OpenCV 影像和 ROS 消息之間的轉換工具
from cv_bridge import CvBridge
# Python 中用於處理信號的庫
import signal
# node 名稱
rospy.init_node('camera_node')
bridge = CvBridge()
# ROS 參數伺服器中獲取相機的索引值
CAMERA_INDEX = rospy.get_param('~index')
# Publisher 名稱
pub = rospy.Publisher(f'camera{CAMERA_INDEX}', Image, queue_size=1)

running = True

def signal_handler(sig, frame):
    global running
    running = False
# XQ 用戶按下 Ctrl+C，會觸發 SIGINT 信號
signal.signal(signal.SIGINT, signal_handler)

cap = cv2.VideoCapture(CAMERA_INDEX)

while running:
    ret, frame = cap.read()
    if not ret:
        continue
    # 資料格式轉換
    image_message = bridge.cv2_to_imgmsg(frame, encoding='passthrough')
    # pub 資料
    pub.publish(image_message)

cap.release()
