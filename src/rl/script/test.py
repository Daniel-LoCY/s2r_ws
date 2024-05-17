#!/usr/bin/env python3

# from tm_msgs.srv import *
# from tm_msgs.msg import *

# f = FeedbackState()

# print(f.tool0_pose)

# import numpy as np
# import tf.transformations as tf  # 使用 transformations 替代 tf.transformations
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# def rotate_vector(vector, axis, angle_degrees):
#     """
#     旋转向量
    
#     参数:
#     - vector: 要旋转的向量 (长度为3的数组或列表)
#     - axis: 旋转轴 (长度为3的数组或列表)
#     - angle_degrees: 旋转角度（以度为单位）
    
#     返回:
#     - 旋转后的向量 (长度为3的数组)
#     """
#     # 将角度从度数转换为弧度
#     angle_radians = np.radians(angle_degrees)
    
#     # 将旋转轴归一化
#     normalized_axis = axis / np.linalg.norm(axis)
    
#     # 构造四元数 (轴角表示法)
#     q = tf.quaternion_about_axis(angle_radians, normalized_axis)
    
#     # 将四元数转换为旋转矩阵
#     rotation_matrix = tf.quaternion_matrix(q)
    
#     # 将向量转换为齐次坐标
#     homogeneous_vector = np.append(vector, 1)
    
#     # 进行旋转
#     rotated_vector = np.dot(rotation_matrix, homogeneous_vector)
    
#     # 返回旋转后的向量（去掉齐次坐标的最后一个分量）
#     return rotated_vector[:3]

# # 绘制向量的函数
# def plot_vectors(vectors, labels, colors, filename):
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     # 设置坐标轴范围
#     ax.set_xlim([-1, 1])
#     ax.set_ylim([-1, 1])
#     ax.set_zlim([-1, 1])

#     # 绘制向量
#     for vector, label, color in zip(vectors, labels, colors):
#         ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color=color)
#         ax.text(vector[0], vector[1], vector[2], label, color=color)

#     # 设置标签
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
    
#     # 保存图像
#     plt.savefig(filename)
#     plt.close()

# # 示例用法
# initial_vector = [1, 0, 0]  # 初始向量
# axis1 = [0, 0, 1]           # 第一次绕 Z 轴旋转
# angle_degrees1 = 90         # 第一次旋转 90 度

# # 第一次旋转
# first_rotated_vector = rotate_vector(initial_vector, axis1, angle_degrees1)
# print("First rotated vector: ", first_rotated_vector)

# # 第二次旋转参数
# axis2 = [1, 0, 0]           # 第二次绕 X 轴旋转
# angle_degrees2 = 180         # 第二次旋转 90 度

# # 第二次旋转
# second_rotated_vector = rotate_vector(first_rotated_vector, axis2, angle_degrees2)
# print("Second rotated vector: ", second_rotated_vector)

# axis2 = [1, 0, 0]           
# angle_degrees2 = -90        

# third_rotated_vector = rotate_vector(second_rotated_vector, axis2, angle_degrees2)
# print("Third rotated vector: ", third_rotated_vector)

# # 绘制初始向量、第一次旋转后的向量和第二次旋转后的向量，并保存为图片
# plot_vectors(
#     [initial_vector, first_rotated_vector, second_rotated_vector, third_rotated_vector], 
#     ['Initial', 'First Rotated', 'Second Rotated', 'Third Rotated'], 
#     ['r', 'g', 'b', 'y'], 
#     'rotated_vector_twice.png'
# )
