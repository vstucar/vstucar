# This file is licensed under MIT license.
# See the LICENSE file in the project root for more information.

cmake_minimum_required(VERSION 2.8.3)
project(car_core)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  car_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES car_core
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

# how to depend from messages
# add_dependencies(target car_msgs_generate_messages_cpp)

catkin_add_nosetests(tests/test_py_common.py)