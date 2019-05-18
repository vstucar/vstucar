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
from matplotlib.colors import hsv_to_rgb

# Параметры перебора вариантов
D_MIN = -5.0                  # Миимальное значение поперечного положения
D_MAX = 5.0                   # Максимальное значение поперечного положения
D_STEP = 2.0                  # Шаг переребора поперечных положений

S_MIN = 20.0                   # Минимальное значение продольного положения
S_MAX = 20.0                  # Максимальное значение продольного положения
S_STEP = 5.0                  # Шаг перебора продольных положений

V_MIN = 10.0                   # Минимальное значение скорости
V_MAX = 10.0                  # Максимальное значение скорости
V_STEP = 0.0                  # Шаг перебора продольных скоростей

T_DEV = 0.0                 # Максимальное отклонение времени от примерной оценки, в долях
                            # ti = [(1-T_DEV)*t_estimate, (1+T_DEV)*t_estimate]
T_STEP = 1                  # Шаг перебора времени

# Параметры расчета
T_CALC_STEP = 0.01          # Шаг интерполяции по времени

# Параметры ограничений
MAX_LON_SPEED = 22          # Maximum longitudinal speed
MIN_LON_SPEED = 0           # Minimum longitudinal speed (0, to remove strange results)
MAX_LON_ACC = 1.5           # Maximum longitudinal acceleration
MIN_LON_DEACC = -1.5        # Minimum longitudinal deacceleration (breaking)
MAX_LAT_ACC = 0.1             # Maximum lateral acceleration
MIN_CURV_RADIUS = 0         # Minimum curvature radius

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

IS_PLOT = True

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
        self.__lattice = {}
        self.__create_endpoints()
        self.__precompute_lattice()
        pass


    """
    def plan(self, state, target):
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

        ax = self.__init_ploting(S0, S1, D0, D1)
        num_lat_trajectories = 0
        num_lon_trajectories = 0
        num_glob_trajectoris = 0

        cnt = 3
        s_step = (S1 - S0)/cnt
        self.plan_stage(0.0, S0, S0+s_step, s_step, S1, D0, D1, np_path, ax)

        plt.show()

        print('Lon trajectories:  {}'.format(num_lon_trajectories))
        print('Lat trajectories:  {}'.format(num_lat_trajectories))
        print('Glob trajectories: {}'.format(num_glob_trajectoris))

    def plan_stage(self, t0, S0, S1, s_step, s_end, D0, D1, np_path, ax):
        num_lat_trajectories = 0
        num_lon_trajectories = 0
        num_glob_trajectoris = 0

        self.plot_lat_dot(ax, t0, D0)
        self.plot_lon_dot(ax, t0, S0)

        for vi in self.__arange(S1[1]-V_DEV, S1[1]+V_DEV, V_STEP):
            for si in self.__arange(S1[0]-S_DEV, S1[0]+S_DEV, S_STEP):
                S1i = (si, vi, S1[2])

                t_estimate = self.__calc_baseline_coefs(S0, S1)
                for ti in self.__arange((1 - T_DEV) * t_estimate, (1 + T_DEV) * t_estimate, T_STEP):
                    lon_trajectory = self.__calc_lon_trajectory(S0, S1i, ti)
                    num_lon_trajectories+=1

                    self.plot_lon(ax, lon_trajectory, t0)

                    for di in self.__arange(D_MIN, D_MAX, D_STEP):
                        D1i = (di, D1[1], D1[2])
                        lat_trajectory = self.__calc_lat_trajectory(D0, D1i, ti)
                        num_lat_trajectories+=1

                        self.plot_lat(ax, lat_trajectory, t0)

                        combined_trajectory = Trajectory2D.from_frenet(lon_trajectory, lat_trajectory)
                        num_glob_trajectoris+=1

                        cost = K_LAT * lat_trajectory.cost + K_LON * lon_trajectory.cost
                        global_trajectory = path_to_global(combined_trajectory, np_path)
                        self.plot_glob(ax, global_trajectory)

                        S0_next = np.array([lon_trajectory.x[-1], lon_trajectory.dx[-1], lon_trajectory.ddx[-1]])
                        D0_next = np.array([lat_trajectory.x[-1], lat_trajectory.dx[-1], lat_trajectory.ddx[-1]])
                        t0_next = t0+lon_trajectory.t[-1]
                        S1_next = S1 + s_step
                        if S1_next[0] <= s_end[0]:
                            self.plan_stage(t0_next, S0_next, S1_next, s_step, s_end, D0_next, D1, np_path, ax)
    """

    # Calc lattice endpoints to make future calculations on regular grid
    def __create_endpoints(self):
        self.__d_endpoints = []
        self.__s_endpoints = []

        for di in self.__arange(D_MIN, D_MAX, D_STEP):
            self.__d_endpoints.append((di, 0, 0))

        for vi in self.__arange(V_MIN, V_MAX, V_STEP):
            for si in self.__arange(S_MIN, S_MAX, S_STEP):
                self.__s_endpoints.append((si, vi, 0))

    # Precompute lattice graph
    def __precompute_lattice(self):
        root = ((0, 15, 0), (0, 0, 0))
        ax = self.__init_ploting((S_MIN, V_MIN, 0), (S_MAX, V_MAX, 0), (D_MIN, 0, 0), (D_MAX, 0, 0))
        self.__precompute_lattice_from_root(root)

        for trajectory in self.__lattice.values():
            self.__plot_trajectory(ax, trajectory)

        plt.show()

    # Precompute tree of the trajectories from one root
    def __precompute_lattice_from_root(self, root):

        S0 = root[0]
        D0 = root[1]

        for S1i in self.__s_endpoints:
            t_estimate = self.__calc_baseline_coefs(S0, S1i)
            print(t_estimate)
            for ti in self.__arange((1 - T_DEV) * t_estimate, (1 + T_DEV) * t_estimate, T_STEP):
                lon_trajectory = self.__calc_lon_trajectory(S0, S1i, ti)

                for D1i in self.__d_endpoints:
                    lat_trajectory = self.__calc_lat_trajectory(D0, D1i, ti)

                    cost = K_LAT * lat_trajectory.cost + K_LON * lon_trajectory.cost
                    combined_trajectory = Trajectory2D.from_frenet(lon_trajectory, lat_trajectory, cost)
                    self.__lattice[(S1i, D1i)] = combined_trajectory

    def __plot_trajectory(self, ax, trajectory):
        self.plot_lon(ax, trajectory.raw_lon)
        self.plot_lat(ax, trajectory.raw_lat)
        self.plot_glob(ax, trajectory)

    def gen_hsv(self, cnt):
        hsv = np.full((cnt, 3), 1.0)
        hsv[:, 0] = np.linspace(0.0, 0.8, cnt)
        return [hsv_to_rgb(color) for color in hsv]

    def plot_glob(self, ax, global_trajectory, color=None):
        if IS_PLOT:
            color = '#ff0000' #color if color is not None else ('#ff0000' if global_trajectory.ok else '#aaaaaa')
            ax[3][0].plot(global_trajectory.pos[:, 0], global_trajectory.pos[:, 1], color=color, alpha=0.5)

    def plot_lat(self, ax, lat_trajectory, t0=0, color=None):
        if IS_PLOT:
            color = '#ff0000' #color if color is not None else ('#ff0000' if lat_trajectory.ok else '#aaaaaa')
            ax[0][0].plot(t0 + lat_trajectory.t, lat_trajectory.x, color=color)
            ax[1][0].plot(t0 + lat_trajectory.t, lat_trajectory.dx, color=color)
            ax[2][0].plot(t0 + lat_trajectory.t, lat_trajectory.ddx, color=color)

    def plot_lon(self, ax, lon_trajectory, t0=0, color=None):
        if IS_PLOT:
            color = '#ff0000' #color if color is not None else ('#ff0000' if lon_trajectory.ok else '#aaaaaa')
            ax[0][1].plot(t0 + lon_trajectory.t, lon_trajectory.x, color=color)
            ax[1][1].plot(t0 + lon_trajectory.t, lon_trajectory.dx, color=color)
            ax[2][1].plot(t0 + lon_trajectory.t, lon_trajectory.ddx, color=color)

    def plot_lat_dot(self, ax, t0, dot):
        ax[0][0].plot([t0], [dot[0]], 'ob')
        ax[1][0].plot([t0], [dot[1]], 'ob')
        ax[2][0].plot([t0], [dot[2]], 'ob')

    def plot_lon_dot(self, ax, t0, dot):
        ax[0][1].plot([t0], [dot[0]], 'ob')
        ax[1][1].plot([t0], [dot[1]], 'ob')
        ax[2][1].plot([t0], [dot[2]], 'ob')

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
        s = np.array([pos_f[0], vel_f[0], acc_f[0]])
        d = np.array([pos_f[1], vel_f[1], acc_f[1]])
        return s, d

    # Calculate the maneuver time estimation based on
    # uniformly accelerated motion
    # s0, s1 - current and target longitudinal states in
    #          Frenet Frame
    def __calc_baseline_coefs(self, s0, s1):
        # v1 = v0 + a*t
        # s1 = s0 + v0*t + a*t^2/2
        t = (2 * (s1[0] - s0[0])) / (s0[1] + s1[1])
        return t
        #a = (s1[1] - s0[1]) / t
        #return t, np.array([0, 0, 0, a / 2, s0[1], 0])

    # arange with closed interval
    # if end border won't include in arange, step will be
    # changed a bit to fit last value exact to the end border
    def __arange(self, start, stop, step):
        eps = 0.001
        if abs(start - stop) < eps:
            return [start]

        cnt = int((stop - start) // step + 1)
        last = start + (cnt - 1) * step
        if abs(last - stop) <= eps:
            return start + np.array(range(cnt)) * step
        else:
            # new_step = (end - start) / cnt
            # mrng = start + np.array(range(cnt + 1)) * new_step
            return np.linspace(start, stop, cnt)

    # Calculate set lateral trajectories
    def __calc_lat_trajectory(self, D0, D1, t1):
        #print('lat')
        #print('D0: {}'.format(D0))
        #print('D1: {}'.format(D1))
        #print('t:  {}'.format(t1))

        t_values = np.arange(0, t1, T_CALC_STEP)
        coefs = quintic.calc_coefs(D0, D1, t1)
        trajectory = Trajectory1D(t_values, *quintic.interpolate(coefs, t_values))

        # Calculate cost
        jerk = quintic.inerpolate_jerk(coefs, t_values)
        trajectory.cost = K_LAT_J * self.__integrate_jerk(jerk) + \
                              K_LAT_T * t1 + \
                              K_LAT_D * (trajectory.x[-1] - D1[0]) ** 2
        trajectory.ok = self.__check_lat_constraints(trajectory)
        return trajectory

    # Calculate a single longitudinal trajectory for given start and end conditions
    def __calc_lon_trajectory(self, S0, S1, t1):
        #print('lon')
        #print('S0: {}'.format(S0))
        #print('S1: {}'.format(S1))
        #print('t:  {}'.format(t1))

        t_values = np.arange(0, t1, T_CALC_STEP)
        coefs = quintic.calc_coefs(S0, S1, t1)
        trajectory = Trajectory1D(t_values, *quintic.interpolate(coefs, t_values))

        # Calculate cost
        jerk = quintic.inerpolate_jerk(coefs, t_values)
        trajectory.cost = K_LON_J * self.__integrate_jerk(jerk) + \
                              K_LON_T * t1 + \
                              K_LON_S * (trajectory.x[-1] - S1[0]) ** 2 + \
                              K_LON_DS * (trajectory.dx[-1] - S1[1]) ** 2
        trajectory.ok = self.__check_lon_constraints(trajectory)
        return trajectory

    def __integrate_jerk(self, jerk):
        return np.sum(np.square(jerk))*T_CALC_STEP

    def __check_lon_constraints(self, trajectory):
        return (trajectory.dx <= MAX_LON_SPEED).all() and (trajectory.dx >= MIN_LON_SPEED).all() and \
               (trajectory.ddx <= MAX_LON_ACC).all() and (trajectory.ddx >= MIN_LON_DEACC).all()

    def __check_lat_constraints(self, trajectory):
        return (np.abs(trajectory.ddx) <= MAX_LAT_ACC).all()

    def __check_combined_constraints(selfs, trajectory):
        # Check curvature radius
        # https://www.math24.net/curvature-plane-curves/
        # We have 2D parametric curve x(t) and y(t), so curvature radius
        # r = 1/k = 1/[(x'y'' - y'x'')/(x'^2 + y'^2)^1.5]
        # We already has all derivatives
        dx = trajectory.dpos[:, 0]
        dy = trajectory.dpos[:, 1]
        ddx = trajectory.ddpos[:, 0]
        ddy = trajectory.ddpos[:, 1]
        curvature = np.abs((dx*ddy - dy*ddx)/np.power(np.square(dx) + np.square(dy), 1.5))
        radius = 1 / curvature
        print('C: min: %f, max: %f' % (np.min(curvature),  np.max(curvature)))
        print('R: min: %f, max: %f' % (np.min(radius), np.max(radius)))
        return (radius >= MIN_CURV_RADIUS).all()

    def __check_obstacles(self, trajectory):
        # TODO:
        for t, pos in zip(trajectory.t, trajectory.pos):
            if self.__map.is_obstacle(t, pos):
                return False
        return True
    
    def __init_ploting(self, S0, S1, D0, D1):

        fig, ax = plt.subplots(4, 2, figsize=(15, 10))
        ax = [
            [plt.subplot2grid((4, 2), (0, 0)), plt.subplot2grid((4, 2), (0, 1))],
            [plt.subplot2grid((4, 2), (1, 0)), plt.subplot2grid((4, 2), (1, 1))],
            [plt.subplot2grid((4, 2), (2, 0)), plt.subplot2grid((4, 2), (2, 1))],
            [plt.subplot2grid((4, 2), (3, 0), colspan=2)],
        ]

        # Lat
        ax[0][0].set_title("Lateral trajectories")
        ax[0][0].set_xlabel("t, s")
        ax[0][0].set_ylabel("d, m")
        ax[0][0].axhline(y=D0[0], color='#000000', linewidth=1)
        ax[0][0].axhline(y=D1[0], color='#000000', linewidth=1)
        ax[1][0].set_xlabel("t, s")
        ax[1][0].set_ylabel("d', m/s")
        ax[1][0].axhline(y=D0[1], color='#000000', linewidth=1)
        ax[1][0].axhline(y=D1[1], color='#000000', linewidth=1)
        ax[2][0].set_xlabel("t, s")
        ax[2][0].set_ylabel("d'', m/s^2")
        ax[2][0].axhline(y=D0[2], color='#000000')
        ax[2][0].axhline(y=D1[2], color='#000000')

        # Lon
        ax[0][1].set_title('Longitudinal trajectories')
        ax[0][1].set_xlabel("t, s")
        ax[0][1].set_ylabel("s, m")
        ax[0][1].axhline(y=S0[0], color='#000000', linewidth=1)
        ax[0][1].axhline(y=S1[0], color='#000000', linewidth=1)
        ax[1][1].set_xlabel("t, s")
        ax[1][1].set_ylabel("s', m/s")
        ax[1][1].axhline(y=S0[1], color='#000000', linewidth=1)
        ax[1][1].axhline(y=S1[1], color='#000000', linewidth=1)
        ax[2][1].set_xlabel("t, s")
        ax[2][1].set_ylabel("s'', m/s^2")
        ax[2][1].axhline(y=S0[2], color='#000000', linewidth=1)
        ax[2][1].axhline(y=S1[2], color='#000000', linewidth=1)

        # Glob
        ax[3][0].set_title('Combined trajectory')
        ax[3][0].set_xlabel('s, m')
        ax[3][0].set_ylabel('d, m')
        ax[3][0].set_aspect('equal', adjustable='box')
        return ax