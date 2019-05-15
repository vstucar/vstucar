# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

# Car state

# Car position and orientation in Cartesian frame
# Note that only YAW rotation and x,y coordinates is meaningful
geometry_msgs/Pose pose

# Car linear forward
std_msgs/Float32 linear_speed