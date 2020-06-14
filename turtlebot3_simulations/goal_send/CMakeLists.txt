cmake_minimum_required(VERSION 2.8.3)
project(goal_send)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  move_base_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES goal_send
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
###########
## Build ##
###########

add_executable(goal_send src/goal_send.cpp)
target_link_libraries(goal_send ${catkin_LIBRARIES})