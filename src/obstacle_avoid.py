#!/usr/bin/env python
import os, sys
import rospy
from geometry_msgs.msg import Pose2D
import time
from time import sleep
import RPi.GPIO as gpio
import pdb
#
dist_front = 0
dist_left = 0
dist_right = 0
## board and PIN allocation
# Setup pi board as BCM
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
## Ultrasonic sensor 
trigger = 4 # blue
echoR    = 17 # white
echoC    = 18 # white
echoL    = 15 # white
#
gpio.setup(trigger,gpio.OUT)  # trigger
gpio.setup(echoR,gpio.IN)      # Echo Right
gpio.setup(echoC,gpio.IN)      # Echo Center
gpio.setup(echoL,gpio.IN)      # Echo Left
## Front  DC motor control
# Motor A, Left Side gpio CONSTANTS
pwm_drive_front_left = 23        # ENA - H-Bridge enable pin (white)
drive_fwd_front_left = 25    # IN1 - Forward Drive (yellow)
drive_rvs_front_left = 24    # IN2 - Reverse Drive (green)
# Motor B, Right Side gpio CONSTANTS
pwm_drive_front_right = 27        # ENB - H-Bridge enable pin (blue)
drive_fwd_front_right = 22    # IN1 - Forward Drive (brown)
drive_rvs_front_right = 10   # IN2 - Reverse Drive (red)
# Setup the gpio pins as Output
gpio.setup(pwm_drive_front_left,gpio.OUT)
gpio.setup(drive_fwd_front_left,gpio.OUT)
gpio.setup(drive_rvs_front_left,gpio.OUT)
gpio.setup(pwm_drive_front_right,gpio.OUT)
gpio.setup(drive_fwd_front_right,gpio.OUT)
gpio.setup(drive_rvs_front_right,gpio.OUT)
# Setup two gpio driver pins as PWM
pwm_front_left = gpio.PWM(pwm_drive_front_left, 1000)
pwm_front_right = gpio.PWM(pwm_drive_front_right, 1000)
#
def fdrive(leftVal, rightVal):
    if leftVal <0: 
        gpio.output(drive_fwd_front_left, False)
        gpio.output(drive_rvs_front_left, True)  
    else:
        gpio.output(drive_fwd_front_left, True) 
        gpio.output(drive_rvs_front_left, False) 
    if rightVal <0: 
        gpio.output(drive_fwd_front_right, False)
        gpio.output(drive_rvs_front_right, True) 
    else:
        gpio.output(drive_fwd_front_right, True)
        gpio.output(drive_rvs_front_right, False) 
    
    pwm_front_left.start(abs(leftVal))
    pwm_front_right.start(abs(rightVal))


## Rear  DC motor control
# Motor A, Left Side gpio CONSTANTS
pwm_drive_rear_left = 26       # ENA - H-Bridge enable pin (yellow)
drive_fwd_rear_left = 19    # IN1 - Forward Drive (yellow)
drive_rvs_rear_left = 13    # IN2 - Reverse Drive (green)
# Motor B, Right Side gpio CONSTANTS
pwm_drive_rear_right = 21        # ENB - H-Bridge enable pin (purple)
drive_fwd_rear_right = 20    # IN1 - Forward Drive (orange)
drive_rvs_rear_right = 16   # IN2 - Reverse Drive (grey)
# Setup the gpio pins as Output
gpio.setup(pwm_drive_rear_left,gpio.OUT)
gpio.setup(drive_fwd_rear_left,gpio.OUT)
gpio.setup(drive_rvs_rear_left,gpio.OUT)
gpio.setup(pwm_drive_rear_right,gpio.OUT)
gpio.setup(drive_fwd_rear_right,gpio.OUT)
gpio.setup(drive_rvs_rear_right,gpio.OUT)
# Setup two gpio driver pins as PWM
pwm_rear_left = gpio.PWM(pwm_drive_rear_left, 1000)
pwm_rear_right = gpio.PWM(pwm_drive_rear_right, 1000)

def rdrive(leftVal, rightVal):
    if leftVal <0: 
        gpio.output(drive_fwd_rear_left, False)
        gpio.output(drive_rvs_rear_left, True)  
    else:
        gpio.output(drive_fwd_rear_left, True) 
        gpio.output(drive_rvs_rear_left, False) 
    if rightVal <0: 
        gpio.output(drive_fwd_rear_right, False)
        gpio.output(drive_rvs_rear_right, True) 
    else:
        gpio.output(drive_fwd_rear_right, True)
        gpio.output(drive_rvs_rear_right, False) 
    
    pwm_rear_left.start(abs(leftVal))
    pwm_rear_right.start(abs(rightVal))

def all_stop():
    gpio.output(drive_fwd_front_left, False) 
    gpio.output(drive_rvs_front_left, False) 
    gpio.output(drive_fwd_front_right, False) 
    gpio.output(drive_rvs_front_right, False) 
    gpio.output(drive_fwd_rear_left, False) 
    gpio.output(drive_rvs_rear_left, False) 
    gpio.output(drive_fwd_rear_right, False) 
    gpio.output(drive_rvs_rear_right, False) 
    pwm_rear_left.stop()
    pwm_rear_right.stop()
    pwm_front_left.stop()
    pwm_front_right.stop()   
    
def callback(msg):
    global dist_front, dist_left, dist_right
    # rospy.loginfo("Received a POSE2D message!")
    rospy.loginfo("distance in cm: [%f,%f,%f]"%(msg.x, msg.y, msg.theta))
    dist_front = round(msg.x, 2)
    dist_right = round(msg.y, 2)
    dist_left = round(msg.theta, 2)    
    #movebot()

def movebot():
    global dist_front, dist_left, dist_right
    while (True):
        print("center =", dist_front, "right = ", dist_right, "left = ", dist_left)   
        print("\n")  
        if dist_front >30 and dist_right >5 and dist_left >5:
            fdrive(30, 30)
            rdrive(30, 30)
            print("Driving forward .. \n")
        elif dist_right <= 5 and dist_left > 5:
            fdrive(10, 40)
            rdrive(10, 40)
            print("Turning Left .. \n")
        elif dist_left <= 5 and dist_right >5:
            fdrive(40, 10)
            rdrive(40, 10)
            print("Turning Right .. \n")
        elif dist_front <=30 and dist_right >5 and dist_left >5:
            # all_stop()        
            # sleep(1)
            # print("Reversing ..\n")
            # fdrive(-45, -45)
            # rdrive(-45, -45)
            # sleep(0.5)  
            # all_stop() 
            # sleep(1)      
            fdrive(50, 10)
            print("Turning right .. \n")
            # sleep(0.5)
            # all_stop() 
            # sleep(1)
        else:
            all_stop()
            sleep(0.5)
            print("All Stop!\n")
    #
def listener():
    rospy.init_node('dist_listener', anonymous=True)
    rospy.Subscriber('distance_msg', Pose2D, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()        
        # now move the robot        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    except KeyboardInterrupt:
        gpio.cleanup()
    finally:  
        gpio.cleanup() 
    
        