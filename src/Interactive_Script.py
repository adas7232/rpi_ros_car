# -*- coding: utf-8 -*-
"""
Created on Sun Apr 12 18:02:36 2020

@author: adas
"""

# dc motor drive 
from gpiozero import PWMOutputDevice
import time
from time import sleep
import RPi.GPIO as gpio
import pdb

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

pdb.set_trace()
indx = 0

def measuredDistance():
    distance_sum_C = 0
    distance_sum_R = 0
    distance_sum_L = 0
    maxLoop = 2
    indx = 0    
    for indx in range(maxLoop):
        # Allow module to settle
        # time.sleep(1)
        # Send 10us pulse to trigger
        gpio.output(trigger, True)
        time.sleep(0.00001)
        gpio.output(trigger, False)   
        # Center           
        while gpio.input(echoC)==0:
            start_C = time.time()    
        while gpio.input(echoC)==1:
            stop_C = time.time() 
        # Right
        while gpio.input(echoR)==0:
            start_R = time.time()    
        while gpio.input(echoR)==1:
            stop_R = time.time()
        # Left
        while gpio.input(echoL)==0:
            start_L = time.time()    
        while gpio.input(echoL)==1:
            stop_L = time.time()   
        # Calculate pulse length
        distance_C = (stop_C-start_C) * 17150 # elapsed_time * 34300 / 2 in cm
        distance_R = (stop_R-start_R) * 17150 
        distance_L = (stop_L-start_L) * 17150 
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)          
        distance_sum_C = distance_sum_C + distance_C
        distance_sum_R = distance_sum_R + distance_R
        distance_sum_L = distance_sum_L + distance_L    
    distance_avg_C = distance_sum_C/maxLoop
    distance_avg_R = distance_sum_R/maxLoop
    distance_avg_L = distance_sum_L/maxLoop    
    return distance_avg_C, distance_avg_L, distance_avg_R
    
def main():
    while True:
        distance_avg_C, distance_avg_L, distance_avg_R = measuredDistance()
        print("Ultrasonic Measurement")
        print("Distance from the center is :", distance_avg_C, " cm")
        print("\n")
        print("Distance from the right is :", distance_avg_R, " cm")
        print("\n")
        print("Distance from the left is :", distance_avg_L, " cm")
        print("\n")
        
        
if __name__ == "__main__":
    """ This is executed when run from the command line """
    main()
