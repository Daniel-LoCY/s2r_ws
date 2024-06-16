#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import signal

rospy.init_node('camera_node')
bridge = CvBridge()

DISPLAY= False

CAMERA_INDEX = rospy.get_param('~index')
pub = rospy.Publisher(f'camera{CAMERA_INDEX}', Image, queue_size=1)

running = True

def signal_handler(sig, frame):
    global running
    running = False

# 註冊信號處理器
signal.signal(signal.SIGINT, signal_handler)

cap = cv2.VideoCapture(CAMERA_INDEX)

while running:
    ret, frame = cap.read()
    if not ret:
        continue

    image_message = bridge.cv2_to_imgmsg(frame, encoding='passthrough')
    pub.publish(image_message)

    if DISPLAY:
        cv2.imshow(f'camera{CAMERA_INDEX}', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
