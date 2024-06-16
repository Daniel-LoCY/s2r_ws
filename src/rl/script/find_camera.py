import cv2

# find all available cameras
CAMERA_INDEX = []
for i in range(10):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        CAMERA_INDEX.append(i)
        cap.release()
print(f'Available cameras: {CAMERA_INDEX}')