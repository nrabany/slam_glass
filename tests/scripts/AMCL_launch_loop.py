from __future__ import print_function
import numpy as np
import roslaunch
import rospy
import time
import subprocess

roscore = subprocess.Popen('roscore')
time.sleep(3)  # wait a bit to be sure the roscore is really launched

for i in range(20):
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nicolas/catkin_ws/src/tests/scripts/AMCL_fast.launch"])
    launch.start()
    rospy.loginfo("started")

    time.sleep(2260) # 1130
    # 3 seconds later
    launch.shutdown()
    time.sleep(10)

roscore.kill()