#!/usr/bin/env python3

import rospy
from tm_msgs.msg import *
from utils import *
            


def get_action():

    req = rlRequest()

    cubeA = tf.get_tf('world', 'object_0')
    cubeB = tf.get_tf('world', 'object_1')
    cubeC = tf.get_tf('world', 'object_2')

    eef_pos = (0, 0, 0)
    eef_quat = (0, 0, 0, 1)

    task = 0

    obj = 0

    target_pos = (0, 0, 0)
    target_quat = (0, 0, 0, 1)

    q_gripper = (0, 0)

    req.cubeA_pos = (cubeA.translation.x, cubeA.translation.y, cubeA.translation.z)
    req.cubeA_quat = (cubeA.rotation.x, cubeA.rotation.y, cubeA.rotation.z, cubeA.rotation.w)

    req.cubeB_pos = (cubeB.translation.x, cubeB.translation.y, cubeB.translation.z)
    req.cubeB_quat = (cubeB.rotation.x, cubeB.rotation.y, cubeB.rotation.z, cubeB.rotation.w)

    req.cubeC_pos = (cubeC.translation.x, cubeC.translation.y, cubeC.translation.z)
    req.cubeC_quat = (cubeC.rotation.x, cubeC.rotation.y, cubeC.rotation.z, cubeC.rotation.w)

    req.eef_pos = eef_pos
    req.eef_quat = eef_quat

    req.task = task

    req.obj = obj

    req.target_pos = target_pos
    req.target_quat = target_quat

    req.q_gripper = q_gripper

    rospy.wait_for_service('/req_action')
    
    action = req_action(req).action

    arm = action[:6]
    gripper = action[6]
    
    return arm, gripper




if __name__ == '__main__':
    rospy.init_node('main_node')
    rospy.loginfo('=== main_node start ===')

    tm = TM()
    tf = TF()

    req_action = rospy.ServiceProxy('/req_action', rl)

    arm, gripper = get_action()

    rospy.loginfo(f'arm: {arm}')
    rospy.loginfo(f'gripper: {gripper}')

    # tm_feedback = rospy.Subscriber('feedback_states', FeedbackState, tm.feedback_callback)

    rospy.spin()