#!/usr/bin/env python
from mpu6050 import mpu6050
from time import sleep
import csv
import pandas as pd
import chart_studio
chart_studio.tools.set_credentials_file(username='adas7232', api_key='HoQUNVkHErthO61JRd5p')
import chart_studio.plotly as py
import plotly.graph_objects as go
import numpy as np
import time
from math import atan2, sqrt, pi
import pdb
#
# reading the hardware
sensor = mpu6050(0x68)
# Setting up the sensor ranges and sensitivities
# # Pre-defined ranges
#     ACCEL_RANGE_2G = 0x00
#     ACCEL_RANGE_4G = 0x08
#     ACCEL_RANGE_8G = 0x10
#     ACCEL_RANGE_16G = 0x18

#     GYRO_RANGE_250DEG = 0x00
#     GYRO_RANGE_500DEG = 0x08
#     GYRO_RANGE_1000DEG = 0x10
#     GYRO_RANGE_2000DEG = 0x18
#
sensor.set_accel_range(0x08) # max |4g|
sensor.read_accel_range(False)
sensor.set_gyro_range(0x08) # max |500 deg/sec|
sensor.read_gyro_range(False)
# accelerometer - m/s^2
# gyro - degree
# temp - degC
timer = time.time()
dt = 0
# 
gyro_ang_x = 0.0
gyro_ang_y = 0.0
gyro_ang_z = 0.0
runtime = 0.0
    
if __name__ == "__main__":   
    try:
        with open('./data/IMUdata.csv', 'w') as outfile:
            outfileWrite = csv.writer(outfile)
            while True:            
                accel_data = sensor.get_accel_data(g=False)
                gyro_data = sensor.get_gyro_data()
                temp = sensor.get_temp()
                #
                acc_x = accel_data['x'] - 1.672041
                acc_y = accel_data['y'] + 0.567635
                acc_z = accel_data['z'] - 10.295464
                #
                gyro_x = gyro_data['x'] + 2.620154
                gyro_y = gyro_data['y'] - 1.139438
                gyro_z = gyro_data['z'] + 0.277876
                #
                #pdb.set_trace() #########################################################
                ## Calculating angle from accelerometer
                roll_acc_deg = atan2(acc_y, sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / pi            
                pitch_acc_deg = atan2(-1*acc_x, sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / pi
                yaw_acc_deg = atan2(acc_z,  sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / pi
                ## Calculating gyro angle
                gyro_ang_x = gyro_ang_x + gyro_x * dt
                gyro_ang_y = gyro_ang_y + gyro_y * dt
                gyro_ang_z = gyro_ang_z + gyro_z * dt
                ## Finally calculating roll pitch and yaw
                roll_deg = 0.9615 * gyro_ang_x + 0.0385 * roll_acc_deg
                pitch_deg = 0.9615 * gyro_ang_y + 0.0385 * pitch_acc_deg
                yaw_deg = 0.9615 * gyro_ang_z + 0.0385 * yaw_acc_deg  

                ## saving data to csv
                # converting time to a meaninful format
                runtime = runtime + dt
                outfileWrite.writerow([runtime, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,temp, roll_deg, pitch_deg, yaw_deg])
                #
                # print("Accelerometer data")
                # print("x: " + str(accel_data['x']))
                # print("y: " + str(accel_data['y']))
                # print("z: " + str(accel_data['z']))

                # print("Gyroscope data")
                # print("x: " + str(gyro_data['x']))
                # print("y: " + str(gyro_data['y']))
                # print("z: " + str(gyro_data['z']))
                print("roll angle= %.2f deg" %roll_deg)
                print("pitch angle %.2f deg" %pitch_deg)
                print("yaw angle %.2f deg" %yaw_deg)
                print("Temp: %.0f C" %temp)  
                sleep(0.5)                 
                dt = time.time() - timer
                timer = time.time()
                
    except KeyboardInterrupt:
        print('interrupted!')
        df = pd.read_csv("./data/IMUdata.csv", skiprows=1, 
                        names = ['timer', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'temp', 
                        'roll_deg', 'pitch_deg', 'yaw_deg'])
        df_acc = df[['acc_x', 'acc_y', 'acc_z']]
        df_gyro = df[['gyro_x', 'gyro_y', 'gyro_z']]
        df_deg = df[['roll_deg', 'pitch_deg', 'yaw_deg']]
        time = df['timer']
        acc_data_x = go.Scatter(x=time, y=df_acc.acc_x.values, name = "acc_x")
        acc_data_y = go.Scatter(x=time, y=df_acc.acc_y.values, name = "acc_y")
        acc_data_z = go.Scatter(x=time, y=df_acc.acc_z.values, name = "acc_z")
        data_acc = [acc_data_x, acc_data_y, acc_data_z]
        py.plot(data_acc, filename = 'IMUacc', auto_open=True)
        # Plotting gyro data
        gyro_data_x = go.Scatter(x=time, y=df_gyro.gyro_x.values, name = "gyro_x")
        gyro_data_y = go.Scatter(x=time, y=df_gyro.gyro_y.values, name = "gyro_y")
        gyro_data_z = go.Scatter(x=time, y=df_gyro.gyro_z.values, name = "gyro_z")
        data_gyro = [gyro_data_x, gyro_data_y, gyro_data_z]
        py.plot(data_gyro, filename = 'IMUgyro', auto_open=True)
        # Sensor fusion acc + gyro
        roll_angle = go.Scatter(x=time, y=df_deg.roll_deg.values, name = "roll_deg")
        pitch_angle = go.Scatter(x=time, y=df_deg.pitch_deg.values, name = "pitch_deg")
        yaw_angle = go.Scatter(x=time, y=df_deg.yaw_deg.values, name = "yaw_deg")
        data_angle = [roll_angle, pitch_angle, yaw_angle]
        py.plot(data_angle, filename = 'calcAngle', auto_open=True)