#!/usr/bin/env python3

from utils import *
import rospy

rospy.init_node('test')

action = req_action(0)

rospy.loginfo(action)