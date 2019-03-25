# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

# Target for motion planner

std_msgs/Header header

# Path motion should be planned for
nav_msgs/Path path

# Target position along the @path
float32 position

# Target linear speed
float32 linear_speed

# Position and orientation of the target
# It should be tangent with path
geometry_msgs/Pose pose