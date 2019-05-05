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


def get_pose(x, y, yaw):
    q = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))
    return Pose(Point(x, y, 0), q)


fake_map = Map()
planner = MotionPlanner(fake_map)

cur_state = CarState(get_pose(0, 0, 0), 8, 0, 0)
cur_state

n_points = 10
dist = 10
path = Path()
path.poses = array_to_path_poses(np.vstack((np.linspace(0, dist, n_points), np.zeros(n_points))).T)
target = MotionPlanningTarget(Header(), path, 15, 0, get_pose(15, 0, 0))

planner.plan(cur_state, target)