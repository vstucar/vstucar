cmake_minimum_required(VERSION 2.8.3)
project(car_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES car_msgs
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
#  DEPENDS system_lib
)