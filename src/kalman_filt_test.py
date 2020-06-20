#!/usr/bin/env python
import pandas as pd
import numpy as np
import rosbag_pandas
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
#
df = rosbag_pandas.bag_to_dataframe('./bagfiles/distance.bag') 
dist_c = df.distance_msg__x
dist_r = df.distance_msg__y
dist_l = df.distance_msg__theta

def kalmanfilt(xp, yp, pq, qq, rr):
    for indx in range(5):
        pq = pq + qq
        Kp = pq/(pq + rr)
        xp = xp + Kp*(yp - xp)    
        pq = (1 - Kp)*pq        
        return xp,pq,Kp

if __name__ == '__main__':
    xp = 80.0
    rr =  3e-3
    pq = 200.0
    qq = 1e-5
    dist_c_filt = np.zeros(len(dist_c))
    Pq = np.zeros(len(dist_c))
    for jdx in range(len(dist_c)-1):
        yp = dist_c[jdx]
        xp, pq, Kp = kalmanfilt(xp, yp, pq, qq, rr)
        dist_c_filt[jdx] = xp
        Pq[jdx] = pq
    
plt.plot(dist_c.values, label="dist_c")
plt.plot(dist_c_filt, label="dist_c_filt")
plt.legend()
plt.savefig('./figures/kfilt_test.png',edgecolor='RED',transparent=False, dpi=600)