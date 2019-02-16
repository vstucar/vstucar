#!/usr/bin/python

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import unittest
import rostest
import rosunit
import numpy as np
from numpy.testing import assert_almost_equal

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from car_core.common import msgs_helpers, geom_helpers

def get_poses_helper(points):
    poses = []
    for p in points:
        pose = PoseStamped()
        pose.pose.position = Point(p[0], p[1], p[2])
        poses.append(pose)
    return poses

class TestMsgsHelpers(unittest.TestCase):

    def test_quaterion_to_array_ok(self):
        q = Quaternion(1,2,3,4)
        arr = msgs_helpers.quaterion_to_array(q)
        assert_almost_equal(arr, np.array([1,2, 3, 4]))
        self.assertTrue(True)

    def test_point_to_array_ok(self):
        p = Point(1,2,3)
        arr = msgs_helpers.point_to_array(p)
        assert_almost_equal(arr, np.array([1,2]))
        self.assertTrue(True)

    def test_path_poses_to_array_ok(self):
        poses = get_poses_helper([[1,2,3],
                                  [4,5,6],
                                  [7,8,9]])
        arr = msgs_helpers.path_poses_to_array(poses)
        assert_almost_equal(arr, np.array([[1,2],
                                           [4,5],
                                           [7,8]]))
        self.assertTrue(True)

    def test_array_to_point_ok(self):
        arr = np.array([1,2])
        point = msgs_helpers.array_to_point(arr)
        self.assertEqual(point, Point(1,2,0))

    def test_array_to_path_poses_ok(self):
        arr = np.array([[1,2],
                        [4,5],
                        [6,7]])
        poses = msgs_helpers.array_to_path_poses(arr)
        poses_true = get_poses_helper([[1,2,0],
                                       [4,5,0],
                                       [6,7,0]])
        self.assertEqual(poses, poses)

class TestGeomHelpers(unittest.TestCase):
    def test_get_closest_path_point_regular(self):
        poses = np.array([[0,0],
                          [1,1],
                          [2,2],
                          [3,3]])
        point = np.array([0.9, 0.9])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 1)

    def test_get_closest_path_point_far(self):
        poses = np.array([[0,0],
                          [1,1],
                          [2,2],
                          [3,3]])
        point = np.array([-1, 3])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 1)

    def test_get_closest_path_point_first(self):
        poses = np.array([[0,0],
                          [1,1],
                          [2,2],
                          [3,3]])
        point = np.array([-1, 1])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 0)

    def test_get_closest_path_point_last(self):
        poses = np.array([[0,0],
                          [1,1],
                          [2,2],
                          [3,3]])
        point = np.array([4, 4])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 3)

    def test_get_closest_path_point_single_point(self):
        poses = np.array([[0,0]])
        point = np.array([4, 4])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 0)

    def test_get_closest_path_point_matching_points(self):
        poses = np.array([[0,0],
                          [1,1],
                          [1,1],
                          [3,3]])
        point = np.array([1.1, 1.1])
        index = geom_helpers.get_closest_path_point(poses, point)
        self.assertEqual(index, 1)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("car_core", 'test_msgs_helpers', TestMsgsHelpers)
    rosunit.unitrun("car_core", 'test_geom_helpers', TestGeomHelpers)