# import roslaunch

# uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)

# launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["src/rl/launch/static_tf.launch"])
# # launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/path/to/launch2.launch"])

# launch1.start()
# # launch2.start()


#!/usr/bin/env python3
from utils import *
import rospy

rospy.init_node('test')

action = req_action(0)

rospy.loginfo(action)