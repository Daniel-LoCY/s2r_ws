#!/usr/bin/env python3

import rospy
from tm_msgs.msg import *
from utils import *

rospy.init_node('main_node')
rospy.loginfo('=== main_node start ===')

tm = TM()
tf = TF()

tm_feedback = rospy.Subscriber('feedback_states', FeedbackState, tm.feedback_callback)

rospy.spin()