#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from random import random
from random import seed
#
def callback(msg):
    rospy.loginfo("Received a POSE2D message!")
    rospy.loginfo("distance in cm: [%f,%f,%f]"%(msg.x, msg.y, msg.theta))    
    #
def listener():
    rospy.init_node('dist_listener', anonymous=True)
    rospy.Subscriber('distance_msg', Pose2D, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()