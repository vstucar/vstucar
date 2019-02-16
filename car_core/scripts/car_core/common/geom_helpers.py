#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import numpy as np
import math
from car_core.common import msgs_helpers
import tf

def get_closest_path_point(path, point):
    """
    Return index of the point in path closest to given
    Args:
        path  (numpy points)
        point (numpy point)
    """
    return np.argmin(np.linalg.norm(path - point, axis=1))

def vector_direction_to_quaternion(vector):
    """
    Convert direction of the 2D vector to
    the quaternion. Euler: (yaw, 0, 0)
    Args:
        vector (numpy point)
    """
    yaw = math.atan2(vector[1], vector[0])
    return tf.transformations.quaternion_from_euler(0, 0, yaw)
