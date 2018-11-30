#!/usr/bin/python

# Remove LRF noise caused by computer lid
# Narrow the LRF scan range from [0,1080] to [10, 1070]

import roslib 
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan



def remoNoise(laserScan):
    staNum = 10     # 0 < staNum < endNum, the staNum-th beam is included
    endNum = 1070  # staNum < endNum < 1080, the endNum-th beam is included
    
    laserScan.intensities = laserScan.intensities[staNum: endNum+1] # both ends included
    laserScan.ranges = laserScan.ranges[staNum: endNum+1]
    laserScan.angle_min = (staNum-540) * laserScan.angle_increment
    laserScan.angle_max = (endNum-540) * laserScan.angle_increment
    
    global pub
    pub.publish(laserScan)
    
if __name__ == "__main__":
    rospy.init_node("remove_LRFnoise")
    global pub
    pub = rospy.Publisher("/scan", LaserScan, queue_size = 10)
    rospy.Subscriber("/first", LaserScan, remoNoise)
    rospy.spin()
