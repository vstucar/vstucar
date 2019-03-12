#!/usr/bin/python
#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import numpy as np
import tf
import tf.transformations
from car_msgs.msg import MotionPlanningTarget, CarState
from car_core.common import msgs_helpers
from frenet import FrenetFrame
from trajectory import Trajectory

class MotionPlanner:
    """
    Performs motion planning using quintic polynoms
    """
    def __init__(self):
        pass

    def plan(self, state, target):
        """
        Plan path
        :param state: (car_msgs/CarState) current pose of the car
        :param target: (car_msgs/MotionPlanningTarget) next target from behaviour planner
        :return: optimal path in car's state space
        """
        if state is None or target is None:
            return None

        # Calc current Cartesian position, Cartesian velocity, and orientation (yaw)
        pos = msgs_helpers.point_to_array(state.pose.position)
        q   = msgs_helpers.quaternion_to_array(state.pose.quaternion)
        m   = tf.transformations.quaternion_matrix(q)
        vel = np.matmul(m, [1,0,0,0])*state.linear_speed
        yaw = tf.transformations.euler_from_quaternion(q)[2]

        # Find closest point on path
        np_path = msgs_helpers.path_poses_to_array(target.path)
        start_index = geom_helpers.get_closest_path_point(np_path, pos)

        # Calc position and velocity in Frenet frame
        frenet_frame = FrenetFrame(0, np_path[0], np_path[1])
        pos_frenet = frenet_frame.point_to(pos)
        vel_frenet = frenet_frame.vector_to(vel)

        s1 = pos_frenet[0]   # initial lon position in frenet frame
        ds1 = vel_frenet[0]  # initial lon position in frenet frame
        d1 = pos_frenet[1]   # initial lat position in frenet frame
        dd1 = vel_frenet[1]  # initial lat speed in frenet frame

        # Estimate
        # Estimate

        return None