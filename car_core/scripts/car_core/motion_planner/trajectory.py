#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import numpy as np
import sys


class Trajectory1D:
    """
    Store trajectory description: position, speed, acceleration of some 
    argument (time, covered arc length, etc)
        y(x), dy/dx(x), d2y/dx^2(x) 
    """
    def __init__(self, t, x, dx, ddx):
        """
        :param t: list of x (argument) points
        :param x: list of y(x) points
        :param dx: list of dy/dx(x) points
        :param ddx: list if d2y/dx^2(x) points
        """
        
        if len(t) != len(x) or len(t) != len(dx):
            raise ValueError('Arrays should be same length')
        
        self.x = x
        self.dx = dx
        self.ddx = ddx
        self.t = t
        self.cost = sys.float_info.max

    def len(self):
        return len(self.t)


class Trajectory2D:
    """
    Store both lon/lat or x/y trajectories
    (position, speed, acceleration) of time
    """

    @staticmethod
    def from_frenet(lon, lat):
        """
        Creates Trajectory2D of pair of trajectories in Frenet Frame
        Args:
            lon (Trajectory1D): longitudinal trajectory
            lat (Trajectory1D): lateral trajectory
        Returns: Combined trajectory
        """
        if len(lat.t) != len(lon.t):
            raise ValueError('Arrays should be same length')

        pos = np.vstack((lon.x, lat.x)).T          # 2D Position
        dpos = np.vstack((lon.dx, lat.dx)).T       # 2D Velocity (dpos/dt)
        ddpos = np.vstack((lon.ddx, lat.ddx)).T    # 2D Acceleration (d2pos/dt&^2)
        return Trajectory2D(lon.t, pos, dpos, ddpos)

    def __init__(self, t, pos, dpos, ddpos):
        """
        Creates Trajectory2D of raw data
        Args:
            t (numpy array): Times
            pos (2d numpy array): Positions of time
            dpos (2d numpy array): Velocities (dpos/dt) of time
            ddpos (2d numpy array): Accelerations (d2pos/dt^2) of time
        """
        self.t = t
        self.pos = pos
        self.dpos = dpos
        self.ddpos = ddpos
