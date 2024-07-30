#!/usr/bin/env python3

import rospy
# TM手臂通訊包
from tm_msgs.msg import *
from utils import *
            


def get_action():
    # class 實體化
    req = rlRequest()
    #------------------------------------------
    # Cube Pose 單位mm 
    cubeA = tf.get_tf('world', 'object_0')
    cubeB = tf.get_tf('world', 'object_1')
    cubeC = tf.get_tf('world', 'object_2')
    #------------------------------------------
    eef_pos = (0, 0, 0)
    eef_quat = (0, 0, 0, 1)

    task = 0

    obj = 0

    target_pos = (0, 0, 0)
    target_quat = (0, 0, 0, 1)

    q_gripper = (0, 0)
    #class 變數 list float32
    #------------------------------------------
    req.cubeA_pos = (cubeA.translation.x, cubeA.translation.y, cubeA.translation.z)
    req.cubeA_quat = (cubeA.rotation.x, cubeA.rotation.y, cubeA.rotation.z, cubeA.rotation.w)
    #------------------------------------------
    req.cubeA_pos = (0, 0 ,0)
    req.cubeA_quat = (0, 0, 0, 1)
    req.cubeB_pos = (0, 0 ,0)
    req.cubeB_quat = (0, 0, 0, 1)
    req.cubeC_pos = (0, 0 ,0)
    req.cubeC_quat = (0, 0, 0, 1)
    #------------------------------------------
    req.cubeB_pos = (cubeB.translation.x, cubeB.translation.y, cubeB.translation.z)
    req.cubeB_quat = (cubeB.rotation.x, cubeB.rotation.y, cubeB.rotation.z, cubeB.rotation.w)

    req.cubeC_pos = (cubeC.translation.x, cubeC.translation.y, cubeC.translation.z)
    req.cubeC_quat = (cubeC.rotation.x, cubeC.rotation.y, cubeC.rotation.z, cubeC.rotation.w)
    #------------------------------------------
    req.eef_pos = eef_pos
    req.eef_quat = eef_quat

    req.task = task

    req.obj = obj

    req.target_pos = target_pos
    req.target_quat = target_quat

    req.q_gripper = q_gripper
    
    rospy.wait_for_service('/req_action')
    # XQ .action
    # 模型的輸出資訊
    action = req_action(req).action

    arm = action[:6]
    gripper = action[6]
    
    return arm, gripper


# speed=80
# m=100
# mtomm=100


if __name__ == '__main__':
    # 初始化了一個名為 main_node 的 ROS 節點，並輸出節點啟動的信息。
    rospy.init_node('main_node')
    rospy.loginfo('=== main_node start ===')
    # 手臂會用到的Function 的class 、 ROS TF會用到的Function 的class
    tm = TM()
    tf = TF()
    # 傳送OB資訊，取得下一步Action
    # rl:rl.srv。
    req_action = rospy.ServiceProxy('/req_action', rl)
    # 手臂目前的資訊
    # FeedbackState：tm_msgs.msg  feedback_callback：utils.py
    tm_feedback = rospy.Subscriber('feedback_states', FeedbackState, tm.feedback_callback)


    while True:
        arm, gripper = get_action()

        rospy.loginfo(f'arm: {arm}')
        rospy.loginfo(f'gripper: {gripper}')

        rospy.loginfo(tm.tool_x)
        
        
        #
        # cmd = f"PTP(\"CPP\",{arm[0]},{arm[1]},{arm[2]},{arm[3]},{arm[4]},{arm[5]}, {speed}, {m}, 0, false)"
        
        # if(gripper>=0):
        #     tm.send_gripper_client(True)
        # else:
        #     tm.send_gripper_client(False)

        rospy.sleep(2)
    
    #保持運行
    # rospy.spin()