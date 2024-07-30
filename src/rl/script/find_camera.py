#!/usr/bin/env python3
# 找相機的port
import cv2

# find all available cameras
CAMERA_INDEX = []
for i in range(100):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        CAMERA_INDEX.append(i)
        cap.release()
print(f'Available cameras: {CAMERA_INDEX}')