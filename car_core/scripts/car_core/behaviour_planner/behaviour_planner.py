#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from car_core.common import msgs_helpers, geom_helpers
from car_msgs.msg import MotionPlanningTarget

WORLD_FRAME_ID = 'map'
class BehaviourPlanner:
    """
    Performs behaviour planning.
    For now just return @global_path with waypoint on it
    """

    def __init__(self, waypoints_gap, ends_gap, target_speed):
        """
        Create behaviour planner
        Args:
            waypoints_gap - minimum distance between waypoints
            ends_gap      - reference_path end margins, before current point,
                            after next waypoint
            target_speed  - constant target speed, will set to every target
        """
        self._waypoints_gap = waypoints_gap
        self._ends_gap = ends_gap
        self._target_speed = target_speed
        self._current_waypoint_index = None

    def plan(self, global_path, car_pose):
        """
        Do planning
        Args:
            global_path (nav_msgs/Path) - global target path
            car_pose (geometry_msgs/PoseStamped) - current car pose
        Returns:
            new waypoint:    car_msgs/MotionPlnningTarget
            no new waypoint: None
        """
        if global_path == None or len(global_path.poses) < 2 or car_pose == None:
            return None

        # Convert path and position to np
        np_path = msgs_helpers.path_poses_to_array(global_path.poses)
        np_point = msgs_helpers.point_to_array(car_pose.pose.position)

        # Find closest point on path
        closest_index = geom_helpers.get_closest_path_point(np_path, np_point)

        # If we reach waypoint, generate next waypoint
        if self._current_waypoint_index == None or closest_index >= self._current_waypoint_index:
            dist, next_waypoint_index = self._get_forward_point_index(np_path, closest_index, self._waypoints_gap)
            print(closest_index, next_waypoint_index)
            self._current_waypoint_index = next_waypoint_index

            # Path
            target = MotionPlanningTarget()
            target.path = Path()
            _, first_path_index = self._get_forward_point_index(np_path, closest_index, self._ends_gap, forward=False)
            _, end_path_index = self._get_forward_point_index(np_path, next_waypoint_index, self._ends_gap)
            target.path.poses = global_path.poses[first_path_index:end_path_index+1]
            target.speed = self._target_speed

            # Position (distance) along the path
            target.position = dist

            # Pose tanget with path
            target.pose.position = global_path.poses[next_waypoint_index].pose.position
            target.pose.orientation = msgs_helpers.array_to_quaternion(self._get_tanget(np_path, next_waypoint_index))

            now = rospy.Time.now()
            target.header.frame_id = WORLD_FRAME_ID
            target.header.stamp = now
            target.path.header.frame_id = WORLD_FRAME_ID
            target.path.header.stamp = now
            return target
        return None

    # Find index of the next point which is @minimum_gap away
    # from starting point along path.
    # forward - look forward or backward
    # Returns actual distance and index
    def _get_forward_point_index(self, path, start_index, minimum_gap, forward = True):
        i = start_index + 1
        dist = 0
        while (i < len(path) and forward) or (i >= 0 and not forward):
            dist += np.linalg.norm(path[i] - path[i-1])
            if dist >= minimum_gap:
                return dist, i
            i = i + (1 if forward else -1)
        return dist, (len(path)-1) if forward else 0

    def _get_tanget(self, np_path, index):
        if index > 0 and index < len(np_path) - 1:
            return geom_helpers.vector_direction_to_quaternion(np_path[index+1] - np_path[index-1])
        elif index > 0:
            return geom_helpers.vector_direction_to_quaternion(np_path[index] - np_path[index-1])
        elif index < len(np_path) - 1:
            return geom_helpers.vector_direction_to_quaternion(np_path[index+1] - np_path[index])
        else:
            rospy.logfatal("This exception cannot be raised")
            raise Exception("This exception cannot be raised")
