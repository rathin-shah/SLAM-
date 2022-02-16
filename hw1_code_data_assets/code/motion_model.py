'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

import sys
import numpy as np
import math


class MotionModel:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """
    def __init__(self):
        """
        TODO : Tune Motion Model parameters here
        The original numbers are for reference but HAVE TO be tuned.
        """
        self._alpha1 = 0.0001
        self._alpha2 = 0.0001
        self._alpha3 = 0.001
        self._alpha4 = 0.001


    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """
        """
        TODO : Add your code here
        """

        x_bar = u_t0[0]
        y_bar = u_t0[1]
        theta_bar = u_t0[2]
        x_bar_dash = u_t1[0]
        y_bar_dash = u_t1[1]
        theta_bar_dash = u_t1[2]
        x_t1 = np.zeros(3)
        delta_rot_1= math.atan2((y_bar_dash-y_bar),(x_bar_dash-x_bar))-theta_bar

        delta_trans= math.sqrt(((x_bar-x_bar_dash)**2) + (y_bar-y_bar_dash)**2)
        delta_rot_2 = theta_bar_dash - theta_bar -delta_rot_1

        term1 = (self._alpha1*(delta_rot_1**2)) + (self._alpha2 *(delta_trans**2)) 
        term2 = (self._alpha3*(delta_trans**2)) + (self._alpha4*(delta_rot_1**2)) + (self._alpha4*(delta_rot_2**2))
        term3 = (self._alpha1*(delta_rot_2**2)) + (self._alpha2*(delta_trans**2))

        delta_rot_1_bar = delta_rot_1 - np.random.normal(0,np.sqrt(term1))
        delta_trans_bar = delta_trans - np.random.normal(0,np.sqrt(term2))
        delta_rot_2_bar = delta_rot_2 - np.random.normal(0,np.sqrt(term3))

        
        x_t1[0] = x_t0[0] + delta_trans_bar*math.cos(x_t0[2] + delta_rot_1_bar)
        x_t1[1] = x_t0[1] + delta_trans_bar*math.sin(x_t0[2] + delta_rot_1_bar)
        x_t1[2] = x_t0[2] + delta_rot_1_bar + delta_rot_2_bar
        return x_t1

    def update_vectorized(self, u_t0, u_t1, X_bar):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """
        """
        TODO : Add your code here
        """

        x_bar = u_t0[0]
        y_bar = u_t0[1]
        theta_bar = u_t0[2]
        x_bar_dash = u_t1[0]
        y_bar_dash = u_t1[1]
        theta_bar_dash = u_t1[2]
        
        delta_rot_1= math.atan2((y_bar_dash-y_bar),(x_bar_dash-x_bar))-theta_bar

        delta_trans= math.sqrt(((x_bar-x_bar_dash)**2) + (y_bar-y_bar_dash)**2)
        delta_rot_2 = theta_bar_dash - theta_bar -delta_rot_1

        term1 = (self._alpha1*(delta_rot_1**2)) + (self._alpha2 *(delta_trans**2)) 
        term2 = (self._alpha3*(delta_trans**2)) + (self._alpha4*(delta_rot_1**2)) + (self._alpha4*(delta_rot_2**2))
        term3 = (self._alpha1*(delta_rot_2**2)) + (self._alpha2*(delta_trans**2))

        delta_rot_1_bar = delta_rot_1 - np.random.normal(0,np.sqrt(term1))
        delta_trans_bar = delta_trans - np.random.normal(0,np.sqrt(term2))
        delta_rot_2_bar = delta_rot_2 - np.random.normal(0,np.sqrt(term3))

        x_t1 = np.zeros(X_bar.shape)
        x_t1[:,0] = X_bar[:,0] + delta_trans_bar*np.cos(X_bar[:,2] + delta_rot_1_bar)
        x_t1[:,1] = X_bar[:,1] + delta_trans_bar*np.sin(X_bar[:,2] + delta_rot_1_bar)
        x_t1[:,2] = X_bar[:,2] + delta_rot_1_bar + delta_rot_2_bar
        return x_t1
if __name__=="__main__":
    pass
