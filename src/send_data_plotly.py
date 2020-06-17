#!/usr/bin/env python
# Import required Python libraries
import chart_studio
chart_studio.tools.set_credentials_file(username='adas7232', api_key='HoQUNVkHErthO61JRd5p')
import chart_studio.plotly as py
import plotly.graph_objects as go
import time
import numpy as np
from numpy import random

data1 = np.loadtxt("./bagfiles/data.txt", delimiter=',', skiprows=1,  unpack=True)
trace0 = go.Scatter(
    x=data1[0,:],
    y=data1[1,:]
)
trace1 = go.Scatter(
    x=data1[0,:],
    y=data1[2,:]
)
trace2 = go.Scatter(
    x=data1[0,:],
    y=data1[3,:]
)
data = [trace0, trace1, trace2]

py.plot(data, filename = 'adas_test', auto_open=True)
