'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

from tkinter import Y
import numpy as np
import math
import time
from matplotlib import pyplot as plt
# from scipy.stats import norm

from map_reader import MapReader


class SensorModel:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """
    def __init__(self, occupancy_map):
        """
        TODO : Tune Sensor Model parameters here
        The original numbers are for reference but HAVE TO be tuned.
        
        """
        # self._z_hit = 1  # 0.1 ~ 10
        # self._z_short = 0.18  # 0.01 ~ 1
        # self._z_max = 0.15  # 0.01 ~ 1
        # self._z_rand = 800  # 10 ~ 1000

        # self._sigma_hit = 100
        # # lambda_short is an intrinsic parameter of the sensor model, for exponential noise
        # self._lambda_short = 0.1

        # # Used in p_max and p_rand, optionally in ray casting
        # self._max_range = 8183

        # # Used for thresholding obstacles of the occupancy map
        # self._min_probability = 0.35
        # self.map = occupancy_map
        # # Used in sampling angles in ray casting
        # self._subsampling = 2

        #Working Tuned
        self._z_hit = 1000
        self._z_short = 0.01
        self._z_max = 0.03
        self._z_rand = 100000

        self._sigma_hit = 250
        self._lambda_short = 0.01

        # Used in p_max and p_rand, optionally in ray casting
        self._max_range = 8183

        # Used for thresholding obstacles of the occupancy map
        self._min_probability = 0.35
        self.map = occupancy_map
        # Used in sampling angles in ray casting
        self._subsampling = 2
       
       #Origial
        # self._z_hit = 1
        # self._z_short = 0.1
        # self._z_max = 0.1
        # self._z_rand = 100

        # self._sigma_hit = 50
        # self._lambda_short = 0.1

        # # Used in p_max and p_rand, optionally in ray casting
        # self._max_range = 1000

        # # Used for thresholding obstacles of the occupancy map
        # self._min_probability = 0.35
        # self.map = occupancy_map
        # # Used in sampling angles in ray casting
        # self._subsampling = 2
    def phit(self,z_t1,z_t1_star):
        if 0 <= z_t1 <= self._max_range:
            gaussian = (math.exp(-(z_t1 - z_t1_star)**2 / (2 * self._sigma_hit**2)))/ math.sqrt(2 * math.pi * self._sigma_hit**2)
            return gaussian
        else:
            return 0.0

    def pshort(self,z_t1,z_t1_star):
        if 0 <= z_t1 <= z_t1_star:
            eta = 1 / (1 - math.exp(-self._lambda_short * z_t1_star))
            exp = eta * self._lambda_short * math.exp(-self._lambda_short* z_t1)
            return exp
        else:
            return 0.0
    
    def pmax(self,z_t1):
        if z_t1 == self._max_range:
            return 1.0
        else:
            return 0.0 

    def p_rand(self, z_t1):
        if 0 <= z_t1 < self._max_range:
            return 1.0 / self._max_range
        else:
            return 0.0       

    def raycast(self, rob_x,rob_y,rob_theta,laser_angle,step):
        x_laser= 25.0 * np.cos(rob_theta)
        y_laser = 25.0 * np.sin(rob_theta)
        _x = np.round((rob_x + x_laser) / 10.0)
        _y = np.round((rob_y + y_laser) / 10.0)
        rob_angle = np.radians(laser_angle)     
        final_laser_angle = rob_theta + rob_angle
        x_start = _x
        y_start = _y
        x_final = x_start
        y_final = y_start
        #while 0 < x_final < self.map.shape[1] and 0 < y_final < self.map.shape[0] and abs(self.map[y_final, x_final]) < 0.0000001:

        while 0 < x_final < self.map.shape[1] and 0 < y_final < self.map.shape[0] and abs(self.map[int(y_final), int(x_final)]) < self._min_probability:
            x_start += self._subsampling * np.cos(final_laser_angle)
            y_start += self._subsampling * np.sin(final_laser_angle)
            x_final = int(round(x_start))
            y_final = int(round(y_start))
        end_point = np.array([x_final,y_final])
        start_point = np.array([_x,_y])
        distance = np.linalg.norm(end_point-start_point) * 10
        return distance         


    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """g
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        """
        TODO : Add your code here
        """
        q=0
        rob_x = x_t1[:,0]
        rob_y = x_t1[:,1]
        rob_theta = x_t1[:,2]
        q_vector = np.zeros((x_t1.shape[0],1))
        for m in range(x_t1.shape[0]):
            for i in range (-90,90, 10):
                z_t1_star = self.raycast(rob_x[m],rob_y[m],rob_theta[m],i,1)
                z_t1 = z_t1_arr[i+90]
                p1 = self._z_hit * self.phit(z_t1, z_t1_star)
                p2 = self._z_short * self.pshort(z_t1, z_t1_star)
                p3 = self._z_max * self.pmax(z_t1)
                p4 = self._z_rand * self.p_rand(z_t1)
                prob_zt1 = p1 + p2 + p3 + p4
    
                if prob_zt1 > 0:
                    q = q + np.log(prob_zt1)
            q_vector[m] = q
        return np.exp(q_vector)