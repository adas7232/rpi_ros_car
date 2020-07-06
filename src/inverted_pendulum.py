#!/usr/bin/env python
from __future__ import print_function, division
import numpy as np          # Grab all of the NumPy functions
from matplotlib.pyplot import * # Grab MATLAB plotting functions
from control.matlab import *    # MATLAB-like functions
import controlpy
m_rod = 0.68 # kg
m_cart = 0.22 # kg
g = 9.81 # m/s^2
l_rod = 0.203 # meter
# defining inverted pendulum model/dynamics in state space form
a_mat = np.matrix([[0,1,0,0], [0,0, -m_rod*g/m_cart, 0], [0, 0, 0, 1], [0, 0, -(m_rod+m_cart)*g/l_rod/m_cart, 0]])
b_mat = np.matrix([[0], [1/m_cart], [0], [1/l_rod/m_cart]])
c_mat = np.matrix([[1,0,0,0],[0,0,1,0]])
d_mat = np.matrix([[0],[0]])
# defining lqr
q_mat = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
r_mat = np.matrix([1])
gain, X, closedLoopEigVals = controlpy.synthesis.controller_lqr(a_mat, b_mat, q_mat, r_mat)
#
print('The computed gain is:')
print(gain)

print('The closed loop eigenvalues are:')
print(closedLoopEigVals)