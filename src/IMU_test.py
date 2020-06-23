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
#
sensor = mpu6050(0x68)
# accelerometer - m/s^2
# gyro - degree
# temp - degC
try:
    with open('./data/IMUdata.csv', 'w') as outfile:
        outfileWrite = csv.writer(outfile)
        while True:
            accel_data = sensor.get_accel_data(g=False)
            gyro_data = sensor.get_gyro_data()
            temp = sensor.get_temp()
            outfileWrite.writerow([accel_data['x'],accel_data['y'],accel_data['z'], gyro_data['x'],gyro_data['y'],gyro_data['z'],temp])
            print("Accelerometer data")
            print("x: " + str(accel_data['x']))
            print("y: " + str(accel_data['y']))
            print("z: " + str(accel_data['z']))

            print("Gyroscope data")
            print("x: " + str(gyro_data['x']))
            print("y: " + str(gyro_data['y']))
            print("z: " + str(gyro_data['z']))

            print("Temp: " + str(temp) + " C")
            sleep(0.5)
except KeyboardInterrupt:
    print('interrupted!')
    df = pd.read_csv("./data/IMUdata.csv", names=['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 'temp_degc'])
    df_acc = df[['acc_x', 'acc_y', 'acc_z']]
    df_gyro = df[['gyro_x', 'gyro_y', 'gyro_z']]
    x_acc = np.linspace(0, len(df_acc.index), len(df_acc.index))
    acc_data_x = go.Scatter(
    x=x_acc,
    y=df_acc.acc_x.values,
    name = "acc_x"
    )
    acc_data_y = go.Scatter(
    x=x_acc,
    y=df_acc.acc_y.values,
    name = "acc_y"
    )
    acc_data_z = go.Scatter(
    x=x_acc,
    y=df_acc.acc_z.values,
    name = "acc_z"
    )
    data_acc = [acc_data_x, acc_data_y, acc_data_z]
    py.plot(data_acc, filename = 'IMUacc', auto_open=True)
    # Plotting gyro data
    x_gyro = np.linspace(0, len(df_gyro.index), len(df_gyro.index))
    gyro_data_x = go.Scatter(
    x=x_gyro,
    y=df_gyro.gyro_x.values,
    name = "gyro_x"
    )
    gyro_data_y = go.Scatter(
    x=x_gyro,
    y=df_gyro.gyro_y.values,
    name = "gyro_y"
    )
    gyro_data_z = go.Scatter(
    x=x_gyro,
    y=df_gyro.gyro_z.values,
    name = "gyro_z"
    )
    data_gyro = [gyro_data_x, gyro_data_y, gyro_data_z]
    py.plot(data_gyro, filename = 'IMUgyro', auto_open=True)