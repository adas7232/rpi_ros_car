#!/usr/bin/env python
# Import required Python libraries
import time
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Pose2D
from random import random

## board and PIN allocation
# Setup pi board as BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
## Ultrasonic sensor 
TRIGGER = 4 # blue
ECHO_R    = 17 # white
ECHO_C    = 18 # white
ECHO_L    = 15 # white
# Setting up GPIO pins
GPIO.setup(TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(ECHO_R,GPIO.IN)      # Echo Right
GPIO.setup(ECHO_C,GPIO.IN)      # Echo Center
GPIO.setup(ECHO_L,GPIO.IN)      # Echo Left
def measuredDistance():
    pub = rospy.Publisher('distance_msg', Pose2D, queue_size=10)
    rospy.init_node('us_pos_measure', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    pos_info = Pose2D()
    # #
    indx = 0
    while not rospy.is_shutdown():
        # Send 10us pulse to trigger       
        distance_sum = 0
        for indx in range(2):            
            # Send 10us pulse to trigger
            GPIO.output(TRIGGER, True)
            time.sleep(0.00001)
            GPIO.output(TRIGGER, False)              
            while GPIO.input(ECHO_C)==0:
              start = time.time()    
            while GPIO.input(ECHO_C)==1:
              stop = time.time()    
            # Calculate pulse length
            elapsed = stop-start    
            # Distance pulse travelled in that time is time
            # multiplied by the speed of sound (cm/s)
            distancet = elapsed * 34300    
            # That was the distance there and back so halve the value
            distance = distancet / 2
            distance_sum = distance_sum + distance    
        distance_avg = distance_sum/2
        pos_info.x = distance_avg 
        pos_info.y = 10 
        pos_info.theta = 0 
        rospy.loginfo(pos_info)
        pub.publish(pos_info)   
    return distance_avg


if __name__ == '__main__':
    try:
        measuredDistance()
    except rospy.ROSInterruptException:
        pass
