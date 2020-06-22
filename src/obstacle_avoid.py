#!/usr/bin/env python
import os, sys
import rospy
from geometry_msgs.msg import Pose2D, Twist
import time
from time import sleep
import RPi.GPIO as gpio
import pdb
from pykalman import KalmanFilter
# Initialization for flobal variables
dist_front = 0
dist_left = 0
dist_right = 0
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
pwm_front_left = gpio.PWM(pwm_drive_front_left, 100)
pwm_front_right = gpio.PWM(pwm_drive_front_right, 100)
#
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
pwm_rear_left = gpio.PWM(pwm_drive_rear_left, 100)
pwm_rear_right = gpio.PWM(pwm_drive_rear_right, 100)

def drive(front_left, front_right, rear_left, rear_right):
    if front_left <0: 
        gpio.output(drive_fwd_front_left, False)
        gpio.output(drive_rvs_front_left, True)  
    else:
        gpio.output(drive_fwd_front_left, True) 
        gpio.output(drive_rvs_front_left, False) 
    if front_right <0: 
        gpio.output(drive_fwd_front_right, False)
        gpio.output(drive_rvs_front_right, True) 
    else:
        gpio.output(drive_fwd_front_right, True)
        gpio.output(drive_rvs_front_right, False) 
       
    pwm_front_left.start(abs(front_left))
    pwm_front_right.start(abs(front_right))
    # 
    if rear_left <0: 
        gpio.output(drive_fwd_rear_left, False)
        gpio.output(drive_rvs_rear_left, True)  
    else:
        gpio.output(drive_fwd_rear_left, True) 
        gpio.output(drive_rvs_rear_left, False) 
    if rear_right <0: 
        gpio.output(drive_fwd_rear_right, False)
        gpio.output(drive_rvs_rear_right, True) 
    else:
        gpio.output(drive_fwd_rear_right, True)
        gpio.output(drive_rvs_rear_right, False) 
       
    pwm_rear_left.start(abs(rear_left))
    pwm_rear_right.start(abs(rear_right))

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
    
# def callback(msg):
#     global dist_front, dist_left, dist_right
#     # rospy.loginfo("Received a POSE2D message!")
#     rospy.loginfo_throttle(2, "distance in cm: [%f,%f,%f]"%(msg.x, msg.y, msg.theta))
#     dist_front = round(msg.x, 2)
#     dist_right = round(msg.y, 2)
#     dist_left = round(msg.theta, 2)    
    
    #movebot()

def movebot(dist_front, dist_right, dist_left):
    # global dist_front, dist_left, dist_right
    flc = 33 # front left command
    frc = 30 # front right command
    rlc = 33 # rear left command
    rrc = 30 # rear right command
    dist_lim_front = 40
    dist_lim_right = 30
    dist_lim_left = 30
    if dist_front > dist_lim_front and dist_left > dist_lim_left and dist_right > dist_lim_right:        
        print("Just drive forward ...")
        drive(flc, frc, rlc, rrc)                       
    elif dist_front < dist_lim_front or dist_left < dist_lim_left or dist_right < dist_lim_right:
        print("Wait, there is something on my way ...")
        all_stop()
        sleep(2)  
        print("Backing up a little ...")
        drive(-flc, -frc, -rlc, -rrc)
        sleep(0.5)
        all_stop()
        sleep(1.5)      
        print("Thinking about turning: Right or Left? ...")
        if dist_right > dist_left:        
            print("Decided to turn right ...")
            drive(flc+10, -frc-10, rlc+10, -rrc-10)
            sleep(1.5)
            all_stop()
            sleep(2)
        elif dist_left > dist_right:        
            print("Decided to turn left ...")
            drive(-flc-10, frc+10, -rlc-10, rrc+10)
            sleep(1.5)
            all_stop()
            sleep(2)
        else:
            print("Backing up more ...")
            drive(-flc, -frc, -rlc, -rrc)
            sleep(1.5)
            all_stop()
            sleep(1.5)

def kalmanfilt(X0, P0, Q, R, y_meas):
    for indx in range(5):
        P0 = P0 + Q
        Kp = P0/(P0 + R)
        X0 = X0 + Kp*(y_meas - X0)    
        P0 = (1 - Kp)*P0        
        return X0
#
if __name__ == '__main__':
    try:
        rospy.init_node('us_pos_measure', anonymous=True)
        pub1 = rospy.Publisher('distance_msg', Pose2D, queue_size=100)
        pos_info = Pose2D() 
        pub2 = rospy.Publisher('dist_filt_msg', Twist, queue_size=100)
        pos_filt_info = Twist() 
        rate = rospy.Rate(10)          
        Q = 0.5   
        R = 241.64    
        P0 = R
        dist_filt_c = 80.0
        dist_filt_r = 70.0
        dist_filt_l = 70.0        
        while not rospy.is_shutdown():
            distance_c = measuredDistance(echo_c)
            distance_r = measuredDistance(echo_r)
            distance_l = measuredDistance(echo_l)     
            # calculating filtered distance            
            dist_filt_c = kalmanfilt(dist_filt_c, P0, Q, R, distance_c)   
            dist_filt_r = kalmanfilt(dist_filt_r, P0, Q, R, distance_r)
            dist_filt_l = kalmanfilt(dist_filt_l, P0, Q, R, distance_l)
            pos_filt_info.linear.x = dist_filt_c
            pos_filt_info.linear.y = dist_filt_r
            pos_filt_info.linear.z = dist_filt_l
            pub2.publish(pos_filt_info) 
            pos_info.x = distance_c
            pos_info.y = distance_r
            pos_info.theta = distance_l
            rospy.loginfo(pos_info)
            pub1.publish(pos_info) 
            movebot(distance_c, distance_r, distance_l) 
            # 
            rate.sleep()             
        #movebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
    except KeyboardInterrupt:
        gpio.cleanup()
    finally:  
        gpio.cleanup() 
    
        