#!/bin/bash

apt update && apt install git -y
# git clone https://github.com/Daniel-LoCY/s2r_ws.git
# cd s2r_ws/
apt install -y ros-noetic-control-msgs ros-noetic-tf ros-noetic-hardware-interface python3-pip ros-noetic-rviz ros-noetic-cv-bridge 
pip install onnx onnxruntime opencv-python scipy opencv-contrib-python
catkin_make
# /bin/bash sh source devel/setup.bash
# roslaunch rl  rl.launch
