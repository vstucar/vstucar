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
import matplotlib.pyplot as plt
ROAD_WIDTH = 3.5*2          # Ширина дороги - допустимой для езды области
D_MIN = -ROAD_WIDTH/2       # Миимальное значение поперечного положения
D_MAX = ROAD_WIDTH/2        # Максимальное значение поперечного положения
D_STEP = 1                  # Шаг переребора поперечных положений

S_DEV =  10                 # Максимальное отклонение продольного положения вперед/назад
                            # от заданного. si = [s_target - S_DEV, s_target + S_DEV]
S_STEP = 3                  # Шаг перебора продольных положений

T_DEV = 0.25                # Максимальное отклонение времени от примерной оценки
                            # ti = [(1-T_DEV)*t_estimate, (1+T_DEV)*t_estimate]
T_CNT = 4                   # Количество переборов времени

T_CALC_STEP = 0.1           # Шаг интерполяции по времени

# Cost function coefficients
K_LAT_J =  1
K_LAT_T =  1
K_LAT_D =  1
K_LON_J =  1
K_LON_T =  1
K_LON_S =  1
K_LON_DS = 1
K_LON    = 1
K_LAT    = 1

class MotionPlanner:
    """
    Performs motion planning using quintic polynoms
    """
    def __init__(self, map):
        """
        Creates motion planner
        Args:
            map: object which provides access to environment map
        """
        self.__map = map
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
        pos0, vel0, yaw0 = self.__get_state(state.pose, state.linear_speed)
        pos1, vel1, yaw1 = self.__get_state(target.pose, target.linear_speed)

        # Find closest point on path
        np_path = msgs_helpers.path_poses_to_array(target.path.poses)
        # start_index = geom_helpers.get_closest_path_point(np_path, pos0)

        # Calc position and velocity in Frenet frame
        frenet_frame = FrenetFrame(0, np_path[0], np_path[1])
        S0, D0 = self.__get_state_in_frenet_frame(frenet_frame, pos0, vel0, [0, 0])  # Initial lon & lat states
        S1, D1 = self.__get_state_in_frenet_frame(frenet_frame, pos1, vel1, [0, 0])  # Target lon & lat states

        print(S0)
        print(S1)
        print(D0)
        print(D1)

        # Estimate maneuver time
        t_estimate, _ = self.__calc_baseline_coefs(S0, S1)

        # Calc bounds of longitudinal and time variations
        t_min = (1 - T_DEV) * t_estimate
        t_max = (1 + T_DEV) * t_estimate

        optimal_trajectory = None
        min_cost = sys.float_info.max

        fig, ax = plt.subplots(4, 2, figsize=(15, 10))
        ax00 = plt.subplot2grid((4, 2), (0, 0))
        ax10 = plt.subplot2grid((4, 2), (1, 0))
        ax20 = plt.subplot2grid((4, 2), (2, 0))
        ax01 = plt.subplot2grid((4, 2), (0, 1))
        ax11 = plt.subplot2grid((4, 2), (1, 1))
        ax21 = plt.subplot2grid((4, 2), (2, 1))
        ax3  = plt.subplot2grid((4, 2), (3, 0), colspan=2)
        ax00.set_title("Lateral trajectories")
        ax00.set_xlabel("t, s")
        ax00.set_ylabel("d, m")
        ax00.axhline(y=D0[0], color='#000000', linewidth=1)
        ax00.axhline(y=D1[0], color='#000000', linewidth=1)
        ax10.set_xlabel("t, s")
        ax10.set_ylabel("d', m/s")
        ax10.axhline(y=D0[1], color='#000000', linewidth=1)
        ax10.axhline(y=D1[1], color='#000000', linewidth=1)
        ax20.set_xlabel("t, s")
        ax20.set_ylabel("d'', m/s^2")
        ax20.axhline(y=D0[2], color='#000000')
        ax20.axhline(y=D1[2], color='#000000')
        ax01.set_title('Longitudinal trajectories')
        ax01.set_xlabel("t, s")
        ax01.set_ylabel("s, m")
        ax01.axhline(y=S0[0], color='#000000', linewidth=1)
        ax01.axhline(y=S1[0], color='#000000', linewidth=1)
        ax11.set_xlabel("t, s")
        ax11.set_ylabel("s', m/s")
        ax11.axhline(y=S0[1], color='#000000', linewidth=1)
        ax11.axhline(y=S1[1], color='#000000', linewidth=1)
        ax21.set_xlabel("t, s")
        ax21.set_ylabel("s'', m/s^2")
        ax21.axhline(y=S0[2], color='#000000', linewidth=1)
        ax21.axhline(y=S1[2], color='#000000', linewidth=1)
        ax3.set_title('Combined trajectory')
        ax3.set_xlabel('s, m')
        ax3.set_ylabel('d, m')

        # Calculate trajectories
        for ti in np.linspace(t_min, t_max, T_CNT):
            t_values = np.arange(0, ti, T_CALC_STEP)
            lat_trajectories = self.__calc_lat_trajectories(D0, D1, t_values)

            for lat_trajectory in lat_trajectories:
                ax00.plot(lat_trajectory.t, lat_trajectory.x)
                ax10.plot(lat_trajectory.t, lat_trajectory.dx)
                ax20.plot(lat_trajectory.t, lat_trajectory.ddx)

            for lon_trajectory in self.__calc_lon_trajectories(S0, S1, t_values):

                ax01.plot(lon_trajectory.t, lon_trajectory.x)
                ax11.plot(lon_trajectory.t, lon_trajectory.dx)
                ax21.plot(lon_trajectory.t, lon_trajectory.ddx)

                # Combine current lon-trajectory (ti, si) with set of lat-trajectories [(ti, d0), (ti, d1), ...],
                for lat_trajectory in lat_trajectories:
                    combined_trajectory = Trajectory2D.from_frenet(lon_trajectory, lat_trajectory)

                    if not self.__check_combined_constraints(combined_trajectory):
                        continue

                    cost = K_LAT * lat_trajectory.cost + K_LON * lon_trajectory.cost
                    #if cost < min_cost:
                    global_trajectory = path_to_global(combined_trajectory, np_path)
                    ax3.plot(global_trajectory.pos[:, 0], global_trajectory.pos[:, 1], color='#777777', alpha=0.5)
                    #if not self.__check_obstacles(global_trajectory):
                    #    min_cost = cost
                    #    optimal_trajectory = global_trajectory

        plt.show()
        return optimal_trajectory

    # Get the car's pos, vel, yaw from the car_msgs/CarState
    # pos - position (vector)
    # vel - velocity (vector)
    # yaw - orientation angle (scalar)
    def __get_state(self, pose, linear_speed):
        pos = msgs_helpers.point_to_array(pose.position)
        q = msgs_helpers.quaternion_to_array(pose.orientation)
        m = tf.transformations.quaternion_matrix(q)
        vel = (np.matmul(m, [1, 0, 0, 0]) * linear_speed)[:2]
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
    def __calc_baseline_coefs(self, s0, s1):
        # v1 = v0 + a*t
        # s1 = s0 + v0*t + a*t^2/2
        t = (2 * (s1[0] - s0[0])) / (s0[1] + s1[1])
        a = (s1[1] - s0[1]) / t
        return t, np.array([0, 0, 0, a / 2, s0[1], 0])

    def __arange(self, min, max, step):
        return np.arange(min, max + step, step)

    # Calculate set lateral trajectories
    def __calc_lat_trajectories(self, D0, D1, t_values):
        lat_trajectories = []
        for di in self.__arange(D_MIN, D_MAX, D_STEP):
            lat_coefs = quintic.calc_coefs(D0, (di, 0, 0), t_values[-1])
            lat_trajectory = Trajectory1D(t_values, *quintic.interpolate(lat_coefs, t_values))

            if self.__check_lat_constraints(lat_trajectory):
                # Calculate cost
                jerk = quintic.inerpolate_jerk(lat_coefs, t_values)
                lat_trajectory.cost = K_LAT_J * self.__integrate_jerk(jerk) + \
                                      K_LAT_T * t_values[-1] + \
                                      K_LAT_D * (lat_trajectory.x[-1] - D1[0]) ** 2
                lat_trajectories.append(lat_trajectory)

        return lat_trajectories

    # Calculate a single longitudinal trajectory for given start and end conditions
    def __calc_lon_trajectories(self, S0, S1, t_values):
        for si in self.__arange(S1[0]-S_DEV, S1[0]+S_DEV, S_STEP):
            lon_coefs = quintic.calc_coefs(S0, (si, S1[1], 0), t_values[-1])
            lon_trajectory = Trajectory1D(t_values, *quintic.interpolate(lon_coefs, t_values))

            if self.__check_lon_constraints(lon_trajectory):
                # Calculate cost
                jerk = quintic.inerpolate_jerk(lon_coefs, t_values)
                lon_trajectory.cost = K_LON_J * self.__integrate_jerk(jerk) + \
                                      K_LON_T * t_values[-1] + \
                                      K_LON_S * (lon_trajectory.x[-1] - S1[0]) ** 2 + \
                                      K_LON_DS * (lon_trajectory.dx[-1] - S1[1]) ** 2
                yield lon_trajectory

    def __calc_wtf_lon_trajectories(self, S0, S1, t_values):
        lon_coefs=[0, 0, 0, 0, 1, 0]
        lon_trajectory = Trajectory1D(t_values, *quintic.interpolate(lon_coefs, t_values))
        yield lon_trajectory

    def __integrate_jerk(self, jerk):
        return np.sum(np.square(jerk))*T_CALC_STEP

    def __check_lon_constraints(self, trajectory):
        # TODO:
        return True

    def __check_lat_constraints(self, trajectory):
        # TODO:
        return True

    def __check_combined_constraints(selfs, trajectory):
        # TODO:
        return True

    def __check_obstacles(self, trajectory):
        # TODO:
        for t, pos in zip(trajectory.t, trajectory.pos):
            if self.__map.is_obstacle(t, pos):
                return False
        return True
