#!/usr/bin/python
#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import sys
import numpy as np
import tf
import tf.transformations
from car_msgs.msg import MotionPlanningTarget, CarState
from car_core.common import msgs_helpers, geom_helpers
from frenet import FrenetFrame, path_to_global
from trajectory import Trajectory1D, Trajectory2D
import quintic

ROAD_WIDTH = 3.5*2          # Ширина дороги - допустимой для езды области
D_MIN   = -ROAD_WIDTH/2     # Миимальное значение поперечного положения
D_MAX   =  ROAD_WIDTH/2     # Максимальное значение поперечного положения
D_STEP  =  1                # Шаг переребора поперечных положений

S_DEV   =  10               # Максимальное отклонение продольного положения вперед/назад
                            # от заданного. si = [s_target - S_DEV, s_target + S_DEV]
S_STEP  =  3                # Шаг перебора продольных положений

T_DEV   =  0.5              # Максимальное отклонение времени от примерной оценки
                            # ti = [(1-T_DEV)*t_estimate, (1+T_DEV)*t_estimate]
T_CNT   =  4                # Количество переборов времени

S_CALC_STEP = 0.5           # Шаг интерполяции по продольному расстоянию (длине дуги)
T_CALC_STEP = 0.2           # Шаг интерполяции по времени


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
        pos0, vel0, yaw0 = self.__get_current_state(state.pose, state.linear_speed)
        pos1, vel1, yaw1 = self.__get_current_state(target.pose, target.linear_speed)

        # Find closest point on path
        np_path = msgs_helpers.path_poses_to_array(target.path)
        start_index = geom_helpers.get_closest_path_point(np_path, pos0)

        # Calc position and velocity in Frenet frame
        frenet_frame = FrenetFrame(0, np_path[0], np_path[1])
        S0, D0 = self.__get_state_in_frenet_frame(frenet_frame, pos0, vel0, [0, 0])  # Initial lon & lat states
        S1, D1 = self.__get_state_in_frenet_frame(frenet_frame, pos1, vel1, [0, 0])  # Target lon & lat states

        # Estimate maneuver time
        t_estimate, _ = self.__calc_baseline_coefs(S0, S1)

        # Calc bounds of longitudinal and time variations
        s_min = S1[0] - S_DEV
        s_max = S1[0] + S_DEV
        t_min = (1 - T_DEV) * t_estimate
        t_max = (1 + T_DEV) * t_estimate

        optimal_trajectory = None
        min_cost = sys.float_info.max

        # Calculate trajectories
        for si in self.__arange(s_min, s_max, S_STEP):

            # Calc lateral trajectories [(si, d0), (si, d1), ...]
            s_values = self.__arange(0, si, S_CALC_STEP)
            lat_trajectories = []
            for di in self.__arange(D_MIN, D_MAX, D_STEP):
                lat_coefs = quintic.calc_coefs(D0, (di, 0, 0), si)
                lat_trajectory = Trajectory1D(s_values, *quintic.interpolate(lat_coefs, s_values))

                if self.__check_lat_constraints(lat_trajectory):
                    lat_trajectories.append(lat_trajectory)

            # Calc longitudinal trajectories [(si, t0), (si, t1), ...]
            for ti in np.linspace(t_min, t_max, T_CNT):
                t_values = self.__arange(0, ti, T_CALC_STEP)
                lon_coefs = quintic.calc_coefs(S0, (si, 0, 0), ti)
                lon_trajectory = Trajectory1D(t_values, *quintic.interpolate(lon_coefs, t_values))

                if not self.__check_lon_constraints(lon_trajectory):
                    continue

                # Combine current lon-trajectory (si, ti) with set of lat-trajectories [(si, d0), (si, d1), ...],
                # transform back to the Cartesian frame, check constraints, check cost, check obstacles
                for lat_trajectory in lat_trajectories:
                    combined_trajectory = Trajectory2D.from_frenet(lon_trajectory, lat_trajectory)

                    if not self.__check_combined_constraints(combined_trajectory):
                        continue

                    cost = self.__calc_cost(lon_trajectory, lat_trajectory)
                    if cost < min_cost:
                        global_trajectory = path_to_global(combined_trajectory, np_path)
                        if not self.__check_obstacles(global_trajectory):
                            min_cost = cost
                            optimal_trajectory = global_trajectory

        return optimal_trajectory

    # Get the car's pos, vel, yaw from the car_msgs/CarState
    def __get_current_state(self, pose, linear_speed):
        pos = msgs_helpers.point_to_array(pose.position)
        q = msgs_helpers.quaternion_to_array(pose.quaternion)
        m = tf.transformations.quaternion_matrix(q)
        vel = np.matmul(m, [1, 0, 0, 0]) * linear_speed
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        return pos, vel, yaw

    # Calculate lateral and longitudinal states
    # (d, d', d'') and (s, s', s'') in Frenet Frame
    # for given pos, vel, acc in Cartesian frame
    def __get_state_in_frenet_frame(self, frame, pos, vel, acc):
        pos_f = frame.point_to(pos)
        vel_f = frame.vector_to(vel)
        acc_f = frame.vector_to(acc)
        s = (pos_f[0], vel_f[0], acc_f[0])
        d = (pos_f[1], vel_f[1], acc_f[1])
        return s, d

    # Calculate the maneuver time estimation based on
    # uniformly accelerated motion
    # s0, s1 - current and target longitudinal states in
    #          Frenet Frame
    def __calc_baseline_coefs(s0, s1):
        # v1 = v0 + a*t
        # s1 = s0 + v0*t + a*t^2/2
        t = (2 * (s1[0] - s0[0])) / (s0[1] + s1[1])
        a = (s1[1] - s0[1]) / t
        return t, np.array([0, 0, 0, a / 2, s0[1], 0])

    def __arange(min, max, step):
        return np.arange(min, max + step, step)

    def __check_lon_constraints(self, trajectory):
        # TODO:
        return True

    def __check_lat_constraints(self, trajectory):
        # TODO:
        return True

    def __check_combined_constraints(selfs, trajectory):
        # TODO:
        return True

    def __calc_cost(self, lon_trajectory, lat_trajectory):
        # TODO:
        return 0

    def __check_obstacles(self, trajectory):
        # TODO:
        return True