#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from random import random
from random import seed
import chart_studio
chart_studio.tools.set_credentials_file(username='adas7232', api_key='HoQUNVkHErthO61JRd5p')
import chart_studio.plotly as py
import plotly.graph_objects as go
#
global dist_center, dist_right, dist_left
def callback(msg):
    rospy.loginfo("Received a POSE2D message!")
    #rospy.loginfo("distance in cm: [%f,%f,%f]"%(msg.x, msg.y, msg.theta))    
    dist_center = msg.x
    dist_right = msg.y
    dist_left = msg.theta  
     
    #
def listener():
    rospy.init_node('dist_listener', anonymous=True)
    rospy.Subscriber('distance_msg', Pose2D, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':    
    listener()
    # while not rospy.is_shutdown():
    #     py.plot(dist_center, filename = 'robot_distance_center', auto_open=True)
    #     rate.sleep()