#!/usr/bin/env python3

import rospy
from rospy import loginfo
import onnx
import onnxruntime as ort
import numpy as np
import os
from custom_msg.srv import *
import tf.transformations as tf

def get_actions(obs: rlRequest):
    # obs = ["cubeA_quat", "cubeA_pos", "cubeB_pos", "cubeB_quat", "cubeC_pos", "cubeC_quat", "eef_pos", "eef_quat", "_obj", "_task", "target_pos", "target_quat"]
    # YOUR_OBS=np.random.rand(1,43)

    # YOUR_OBS = np.array([([-1.8222e-05,  1.2036e-05, -2.9527e-07,  1.0000e+00, # cubeA_quat
    #                         -5.8000e-01, -3.5353e-01,  1.0450e+00, # cubeA_pos
    #                         -5.8000e-01, 2.9294e-01,  1.0450e+00, # cubeB_pos
    #                         -1.8222e-05,  1.2036e-05, -2.9527e-07, 1.0000e+00, # cubeB_quat
    #                         -5.8000e-01,  8.0893e-02,  1.0450e+00, # cubeC_pos
    #                         -1.8141e-05, 1.2032e-05, -3.7930e-07, 1.0000e+00, # cubeC_quat
    #                         -8.7917e-01, -9.1880e-02,  1.2288e+00, # eef_pos
    #                         -6.4305e-01, -2.6193e-02, -7.6316e-01,  5.8127e-02, # eef_quat
    #                         2.0000e+00, # _obj
    #                         1.0000e+00, # _task
    #                         -1.2000e+00, -5.0000e-01,  1.1200e+00, # target_pos
    #                         0.0000e+00,  0.0000e+00, -7.0711e-01,  7.0711e-01, # target_quat
    #                         3.7833e-02,  3.7690e-02 # q_gripper
    #                     ])])

    obs_list = [obs.cubeA_quat, obs.cubeA_pos, obs.cubeB_pos, obs.cubeB_quat, obs.cubeC_pos, obs.cubeC_quat, obs.eef_pos, obs.eef_quat, tuple([obs.obj]), tuple([obs.task]), obs.target_pos, obs.target_quat, obs.q_gripper]
    YOUR_OBS = np.array([np.concatenate(obs_list)])
    outputs = loaded_model.run(None, {"obs": YOUR_OBS.astype(np.float32)})[0][0]
    return [outputs.tolist()]

def rotate_vector(params: rotate_vecRequest):
    """
    旋轉向量
    
    參數:
    - vector: 要旋轉的向量（長度為3的陣列或列表） [0, 0, 0]
    - axis: 旋轉軸（長度為3的陣列或列表） [0, 0, 1]
    - angle_degrees: 旋轉角度（以度為單位） 90
    
    返回:
    - 旋轉後的向量（長度為3的陣列） [0, 0, 0]
    """
    vector = params.vector
    axis = params.axis
    angle_degrees = params.angle_degrees

    # 將角度從度數轉換為弧度
    angle_radians = np.radians(angle_degrees)
    
    # 將旋轉軸歸一化
    normalized_axis = axis / np.linalg.norm(axis)
    
    # 構造四元數（軸角表示法）
    q = tf.quaternion_about_axis(angle_radians, normalized_axis)
    
    # 將四元數轉換為旋轉矩陣
    rotation_matrix = tf.quaternion_matrix(q)
    
    # 將向量轉換為齊次坐標
    homogeneous_vector = np.append(vector, 1)
    
    # 進行旋轉
    rotated_vector = np.dot(rotation_matrix, homogeneous_vector)
    
    # 返回旋轉後的向量（去掉齊次坐標的最後一個分量）
    return rotated_vector[:3]


if __name__ == '__main__':
    model_path = f"/catkin_ws/models/test.onnx"
    onnx_model = onnx.load(model_path)
    onnx.checker.check_model(onnx_model)
    loaded_model = ort.InferenceSession(model_path)

    rospy.init_node('service')
    rospy.loginfo('service node start')
    rospy.Service('/get_action', rl, get_actions)
    rospy.Service('/rotate_vector', rotate_vec, rotate_vector)
    rospy.spin()