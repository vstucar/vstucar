#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import numpy as np
from geometry_msgs.msg import Point, PoseStamped, Quaternion

def quaternion_to_array(q):
    """Converts geometry_msgs/Quaternion to numpy array"""
    return np.array([q.x, q.y, q.z, q.w])

def point_to_array(p):
    """Converts geometry_msgs/Point to numpy array"""
    return np.array([p.x, p.y])

def path_poses_to_array(poses):
    """"
    Convert array of geometry_msgs/Pose (e.g. from nav_msgs/Path)
    to 2D numpy array
    """
    return np.array([[pose.pose.position.x, pose.pose.position.y] for pose in poses])

def array_to_quaternion(arr):
    return Quaternion(*arr)

def array_to_point(np_point):
    """Convert numpy array to geometry_msgs/Point"""
    return Point(np_point[0], np_point[1], 0)

def array_to_path_poses(np_path):
    """Convert 2D numpy array to array of geometry_msgs/Poses"""
    poses = []
    for np_point in np_path:
        pose = PoseStamped()
        pose.pose.position = array_to_point(np_point)
        poses.append(pose)
    return poses