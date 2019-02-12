#coding=utf-8

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

import rospy

def main():
    rospy.init_node('behaviour_planner_node')
    rospy.loginfo("behaviour_planner_node started")
    rospy.spin()