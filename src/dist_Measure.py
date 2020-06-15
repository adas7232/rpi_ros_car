#!/usr/bin/env python
# Import required Python libraries
import time
import RPi.GPIO as gpio
import rospy
from geometry_msgs.msg import Pose2D
from random import random
import pdb
## board and PIN allocation
# Setup pi board as BCM
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
## Ultrasonic sensor 
trigger = 4 # blue
echo_r    = 17 # white
echo_c    = 18 # white
echo_l    = 15 # white
# Setting up gpio pins
gpio.setup(trigger,gpio.OUT)  # Trigger
gpio.setup(echo_r,gpio.IN)      # echo Right
gpio.setup(echo_c,gpio.IN)      # echo Center
gpio.setup(echo_l,gpio.IN)      # echo Left
def measuredDistance(echo):
    global start_time, stop_time
    # Send 10us pulse to trigger          
    distance_sum = 0    
    maxLoop = 2           
    for indx in range(maxLoop):
        # Allow module to settle and makeit false if not already
        gpio.output(trigger, False)
        time.sleep(0.001)
        # Send 10us pulse to trigger
        gpio.output(trigger, True)
        time.sleep(0.00001)
        gpio.output(trigger, False)   
        # Center                      
        while gpio.input(echo)==0:                    
            start_time = time.time()               
        while gpio.input(echo)==1:              
            stop_time = time.time()               
        # Calculate pulse length
        distance_ini = (stop_time-start_time) * 17150 # elapsed_time * 34300 / 2 in cm             
        distance_sum = distance_sum + distance_ini            
    distance = distance_sum/maxLoop
    return distance

        
def publish_distance():
    pub = rospy.Publisher('distance_msg', Pose2D, queue_size=100)
    rospy.init_node('us_pos_measure', anonymous=True)
    pos_info = Pose2D() 
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        distance_c = measuredDistance(echo_c)
        distance_r = measuredDistance(echo_r)
        distance_l = measuredDistance(echo_l)
        pos_info.x = distance_c
        pos_info.y = distance_r
        pos_info.theta = distance_l
        rospy.loginfo(pos_info)
        pub.publish(pos_info) 
        rate.sleep() 

if __name__ == '__main__':
    try:
        publish_distance()
    except rospy.ROSInterruptException:
        pass
