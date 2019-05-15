#!/usr/bin/python

import numpy as np
import rospy
import tf
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Point, Quaternion

from car_msgs.msg import CarState, MotionPlanningTarget
from car_core.motion_planner.motion_planner import MotionPlanner
from car_core.map.map import Map
from car_core.common.msgs_helpers import *

import time

def get_pose(x, y, yaw):
    q = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))
    return Pose(Point(x, y, 0), q)


fake_map = Map()
planner = MotionPlanner(fake_map)

vel0 = 15
vel1 = 5
dist = 100
n_points = 10

cur_state = CarState(get_pose(0, 0, 0), 15)  # Point, linear speed,

path = Path()
path.poses = array_to_path_poses(np.vstack((np.linspace(0, dist, n_points), np.zeros(n_points))).T)
target = MotionPlanningTarget(Header(), path, get_pose(dist, 0, 0), vel1)

start = time.time()
planner.plan(cur_state, target)
duration = time.time() - start
print('Elapsed time: {}'.format(duration))