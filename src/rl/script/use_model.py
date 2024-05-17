#!/usr/bin/env python3

import numpy as np
import rospy
from custom_msg.srv import *

def req_action(obs):
    rospy.wait_for_service('/get_action')
    get_action = rospy.ServiceProxy('/get_action', rl)
    req = rlRequest()
    req.cubeA_pos = (0, 0, 0)
    req.cubeA_quat = (0, 0, 0, 1)
    req.cubeB_pos = (0, 0, 0)
    req.cubeB_quat = (0, 0, 0, 1)
    req.cubeC_pos = (0, 0, 0)
    req.cubeC_quat = (0, 0, 0, 1)
    req.eef_pos = (0, 0, 0)
    req.eef_quat = (0, 0, 0, 1)
    req.obj = 0
    req.task = 0
    req.target_pos = (0, 0, 0)
    req.target_quat = (0, 0, 0, 1)
    req.q_gripper = (0, 0)
    
    action = get_action(req).action
    arm = action[:6]
    gripper = action[6]
    rospy.loginfo(f'arm: {arm}')
    rospy.loginfo(f'gripper: {gripper}')
    return action

if __name__ == '__main__':
    rospy.init_node('send_obs')
    rospy.wait_for_service('/get_action')
    get_action = rospy.ServiceProxy('/get_action', rl)
    req_action(None)