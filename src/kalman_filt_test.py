#!/usr/bin/env python
import pandas as pd
import numpy as np
import rosbag_pandas
from pykalman import KalmanFilter
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
#
df = rosbag_pandas.bag_to_dataframe('./bagfiles/distance.bag') 
dist_c = df.distance_msg__x
dist_r = df.distance_msg__y
dist_l = df.distance_msg__theta

def kalmanfilt(X0, P0, Q, R, y_meas):
    for indx in range(5):
        P0 = P0 + Q
        Kp = P0/(P0 + R)
        X0 = X0 + Kp*(y_meas - X0)    
        P0 = (1 - Kp)*P0        
        return X0,P0,Kp

if __name__ == '__main__':
    F = 1.0
    H = 1.0
    X0 = 80.0
    Q = 1   
    R = 241.64    
    P0 = R
# Implementing 1-D kalman filter 
    # initializations
    dist_c_filt = np.zeros(len(dist_c))
    Ps = np.zeros(len(dist_c))
    #
    for jdx in range(len(dist_c)-1):
        y_meas = dist_c[jdx]
        X0, P0, Kp = kalmanfilt(X0, P0, Q, R, y_meas)
        dist_c_filt[jdx] = X0
        Ps[jdx] = P0
# Implementing pykalman toolbox    
    F = 1.0
    H = 1.0
    X0 = 80.0
    Q = 10    
    R = 241.64    
    P0 = R
    # initializations
    filtered_state_means = np.zeros(len(dist_c))
    filtered_state_covariances = np.zeros(len(dist_c))
    #
    kf = KalmanFilter(transition_matrices = F, 
                    observation_matrices = H, 
                    transition_covariance = Q, 
                    observation_covariance = R, 
                    initial_state_mean = X0, 
                    initial_state_covariance = P0)
    # Implementing filter update
    for t in range(len(dist_c)):
        if t == 0:
            filtered_state_means[t] = X0
            filtered_state_covariances[t] = P0
        else:
            filtered_state_means[t], filtered_state_covariances[t] = (
            kf.filter_update(
                filtered_state_means[t-1],
                filtered_state_covariances[t-1],
                dist_c[t]
            )
            )
    plt.plot(dist_c.values, label="dist_c")
    plt.plot(dist_c_filt, label="dist_c_filt")
    plt.plot(filtered_state_means, label="dist_c_filt_pykalman")
    plt.legend()
    plt.savefig('./figures/kfilt_test.png',edgecolor='RED',transparent=False, dpi=600)