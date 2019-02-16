#!/usr/bin/python

# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

# This classes wraps working with RViz markers
# to simplify debugging and visualiztion

import math
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from car_core.common import geom_helpers, msgs_helpers

class LinesHelper:
    """
    This helper allows to publish marker with one or more lines
    """

    def __init__(self, topic, color, width = 0.1):
        """
        Create helper
        Args:
            - topic - topic for publishing
            - color - marker's color as RGB arr or tuple
            - width - marker's line width
        """
        self.pub = rospy.Publisher(topic, Marker, queue_size = 1)
        self.marker = Marker()
        self.marker.type = Marker.LINE_LIST
        self.marker.header.frame_id = 'map'
        self.marker.scale = Vector3(width, 0, 0)
        self.marker.color = ColorRGBA(*color)

    def publish(self, lines):
        """
        Publish marker
        Args:
            lines - array of points. Points is represented as a numpy array
        """
        self.marker.points = []
        for line in lines:
            for i in range(len(line)-1):
                self.marker.points.append(msgs_helpers.array_to_point(line[i]))
                self.marker.points.append(msgs_helpers.array_to_point(line[i+1]))

        self.pub.publish(self.marker)

class FigureHelper:
    """
    This helper allow to publish spehere/cube/cylinder marker
    """

    def __init__(self, topic, color, scale, shape=Marker.SPHERE):
        """
        Create helper
        Args:
            - topic - topic for publishing
            - color - marker's color as RGB arr or tuple
            - scale - shape scale
            - shape - marker's shape (Marker.CUBE, Marker.SPHERE, Marker.CYLINDER)
        """
        self.pub = rospy.Publisher(topic, Marker, queue_size = 1)
        self.marker = Marker()
        self.marker.type = shape
        self.marker.header.frame_id = 'map'
        self.marker.scale = Vector3(*scale)
        self.marker.color = ColorRGBA(*color)

    def publish(self, point):
        """
        Publish marker
        Args:
            point - marker's position (geometry_msgs/Point)
        """
        self.marker.pose.position = point
        self.pub.publish(self.marker)
