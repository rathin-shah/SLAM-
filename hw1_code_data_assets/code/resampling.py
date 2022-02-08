'''
    Adapted from course 16831 (Statistical Techniques).
    Initially written by Paloma Sodhi (psodhi@cs.cmu.edu), 2018
    Updated by Wei Dong (weidong@andrew.cmu.edu), 2021
'''

import numpy as np


class Resampling:
    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """
    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    # def multinomial_sampler(self, X_bar):
    #     """
    #     param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    #     param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
    #     """
    #     """
    #     TODO : Add your code here
    #     """
    #     X_bar_resampled =  np.zeros_like(X_bar)
    #     return X_bar_resampled

    def low_variance_sampler(self, X_bar):
        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        """
        TODO : Add your code here
        """
        X_bar_resampled =  np.zeros_like(X_bar)
        M = len(X_bar)
        r = np.random.uniform(0, 1/M)
        wt = X_bar[:,3]
        wt =wt/ wt.sum()
        c=wt[0]
        i = 0
        j = 0
        for m in range(M):
            u = r + (m-1)*(1/M)
            while u > c:
                i=i+1
                c=c+wt[i]
            X_bar_resampled[j]=X_bar[i]
            j=j+1
        return X_bar_resampled
