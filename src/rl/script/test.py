import roslaunch

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch1 = roslaunch.parent.ROSLaunchParent(uuid, ["src/rl/launch/static_tf.launch"])
# launch2 = roslaunch.parent.ROSLaunchParent(uuid, ["/path/to/launch2.launch"])

launch1.start()
# launch2.start()