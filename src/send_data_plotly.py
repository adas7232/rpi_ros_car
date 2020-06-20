#!/usr/bin/env python
# Import required Python libraries
import chart_studio
chart_studio.tools.set_credentials_file(username='adas7232', api_key='HoQUNVkHErthO61JRd5p')
import chart_studio.plotly as py
import plotly.graph_objects as go
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import time
import numpy as np
from numpy import random
import rosbag_pandas

# import os
# os.system("rostopic echo -b ./bagfiles/distance_2020-06-20-00-31-54.bag -p /distance_msg > data.txt")

# data1 = np.loadtxt("./bagfiles/data.txt", delimiter=',', skiprows=1,  unpack=True)
# trace0 = go.Scatter(
#     x=data1[0,:],
#     y=data1[1,:]
# )
# trace1 = go.Scatter(
#     x=data1[0,:],
#     y=data1[2,:]
# )
# trace2 = go.Scatter(
#     x=data1[0,:],
#     y=data1[3,:]
# )
# data = [trace0, trace1, trace2]

# py.plot(data, filename = 'adas_test', auto_open=True)
df = rosbag_pandas.bag_to_dataframe('./bagfiles/distance.bag') # distance_msg__theta  distance_msg__x  distance_msg__y
df_filt = rosbag_pandas.bag_to_dataframe('./bagfiles/filt_distance.bag')
#
plt.figure()
plt.plot(df.distance_msg__x, label="dist_center")
plt.plot(df.distance_msg__y, label="dist_right")
plt.plot(df.distance_msg__theta, label="dist_left")
plt.legend()
plt.savefig('./figures/distance.png',edgecolor='RED',transparent=False, dpi=600)
#
plt.figure()
plt.plot(df_filt.dist_filt_msg__linear_x, label="dist_filt_center")
plt.plot(df_filt.dist_filt_msg__linear_y, label="dist_filt_right")
plt.plot(df_filt.dist_filt_msg__linear_z, label="dist_filt_left")
plt.legend()
plt.savefig('./figures/dist_filt.png',edgecolor='RED',transparent=False, dpi=600)
