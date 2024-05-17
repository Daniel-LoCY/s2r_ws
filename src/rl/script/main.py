#!/usr/bin/env python3

import rospy
from tm_msgs.msg import *
from rospy import loginfo
from utils.utils import *

rospy.init_node('main_node')
loginfo('=== main_node start ===')

tm = TM()
tf = TF()

loginfo(tm.tool_x)

# sub_feedback = rospy.Subscriber('feedback_states', FeedbackState, tm.feedback_callback)
# rospy.spin()