#!/usr/bin/env python3

import rospy
import onnx
import onnxruntime as ort
import numpy as np
from custom_msg.srv import *
# import tf.transformations as tf

def req_action(obs: rlRequest):
    obs_list = [obs.cubeA_quat, obs.cubeA_pos, obs.cubeB_pos, obs.cubeB_quat, obs.cubeC_pos, obs.cubeC_quat, obs.eef_pos, obs.eef_quat, tuple([obs.obj]), tuple([obs.task]), obs.target_pos, obs.target_quat, obs.q_gripper]
    YOUR_OBS = np.array([np.concatenate(obs_list)])
    outputs = loaded_model.run(None, {"obs": YOUR_OBS.astype(np.float32)})[0][0]
    resp = rlResponse()
    resp.action = outputs.tolist()
    return resp

# def rotate_vector(params: rotate_vecRequest):
#     """
#     旋轉向量
    
#     參數:
#     - vector: 要旋轉的向量（長度為3的陣列或列表） [0, 0, 0]
#     - axis: 旋轉軸（長度為3的陣列或列表） [0, 0, 1]
#     - angle_degrees: 旋轉角度（以度為單位） 90
    
#     返回:
#     - 旋轉後的向量（長度為3的陣列） [0, 0, 0]
#     """
#     vector = params.vector
#     axis = params.axis
#     angle_degrees = params.angle_degrees

#     # 將角度從度數轉換為弧度
#     angle_radians = np.radians(angle_degrees)
    
#     # 將旋轉軸歸一化
#     normalized_axis = axis / np.linalg.norm(axis)
    
#     # 構造四元數（軸角表示法）
#     q = tf.quaternion_about_axis(angle_radians, normalized_axis)
    
#     # 將四元數轉換為旋轉矩陣
#     rotation_matrix = tf.quaternion_matrix(q)
    
#     # 將向量轉換為齊次坐標
#     homogeneous_vector = np.append(vector, 1)
    
#     # 進行旋轉
#     rotated_vector = np.dot(rotation_matrix, homogeneous_vector)
    
#     # 返回旋轉後的向量（去掉齊次坐標的最後一個分量）
#     return rotated_vector[:3]


if __name__ == '__main__':
    model_path = "/home/daniel/s2r_ws/models/test.onnx"
    onnx_model = onnx.load(model_path)
    onnx.checker.check_model(onnx_model)
    loaded_model = ort.InferenceSession(model_path)

    rospy.init_node('service')
    # rospy.loginfo('service node start')
    rospy.Service('/req_action', rl, req_action)
    # rospy.Service('/rotate_vector', rotate_vec, rotate_vector)
    rospy.spin()