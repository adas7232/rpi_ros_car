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

# 
df = rosbag_pandas.bag_to_dataframe('./bagfiles/distance.bag') # distance_msg__theta  distance_msg__x  distance_msg__y
df_filt = rosbag_pandas.bag_to_dataframe('./bagfiles/filt_distance.bag')
#
plt.figure()
plt.plot(df.distance_msg__x, label="dist_center")
plt.plot(df.distance_msg__y, label="dist_right")
plt.plot(df.distance_msg__theta, label="dist_left")
# plt.legend()
# plt.savefig('./figures/distance.png',edgecolor='RED',transparent=False, dpi=600)
#
# plt.figure()
plt.plot(df_filt.dist_filt_msg__linear_x, label="dist_filt_center")
plt.plot(df_filt.dist_filt_msg__linear_y, label="dist_filt_right")
plt.plot(df_filt.dist_filt_msg__linear_z, label="dist_filt_left")
plt.legend()
plt.savefig('./figures/dist_filt.png',edgecolor='RED',transparent=False, dpi=600)

# Send data to plotly
x1 = np.linspace(0, len(df.index), len(df.index))
x2 = np.linspace(0, len(df_filt.index), len(df_filt.index))
dist_center = go.Scatter(
    x=x1,
    y=df.distance_msg__x.values,
    name = "dist_center"
)
dist_right = go.Scatter(
    x=x1,
    y=df.distance_msg__y.values,
    name = "dist_right"
)
dist_left = go.Scatter(
    x=x1,
    y=df.distance_msg__theta.values,
    name = "dist_left"
)
dist_center_filt = go.Scatter(
    x=x2,
    y=df_filt.dist_filt_msg__linear_x.values,
    name = "dist_center_filt"
)
dist_right_filt = go.Scatter(
    x=x2,
    y=df_filt.dist_filt_msg__linear_y.values,
    name = "dist_right_filt"
)
dist_left_filt = go.Scatter(
    x=x2,
    y=df_filt.dist_filt_msg__linear_z.values,
    name = "dist_left_filt"
)
# layout = go.Layout(
#         yaxis=YAxis(
#             title="yy"         
#         ),
#         xaxis=XAxis(
#             title="xx"            
#         ),
#         hovermode="closest"
#     )
data = [dist_center, dist_right, dist_left, dist_center_filt, dist_right_filt, dist_left_filt]
# sending data 
py.plot(data, filename = 'US_sensor_data', auto_open=True)