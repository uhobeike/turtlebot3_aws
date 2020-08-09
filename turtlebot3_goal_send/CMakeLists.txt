cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_goal_send)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
###########
## Build ##
###########

#add_executable(goal_send src/goal_send.cpp)
#target_link_libraries(goal_send ${catkin_LIBRARIES})

#add_executable(4goal_send src/4goal_send.cpp)
#add_dependencies(4goal_send ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(4goal_send ${catkin_LIBRARIES})

#add_executable(goal_control_key src/goal_control_key.cpp)
#target_link_libraries(goal_control_key ${catkin_LIBRARIES})

add_executable(goal_waypoint_control_key src/goal_waypoint_control_key.cpp)
target_link_libraries(goal_waypoint_control_key ${catkin_LIBRARIES})

add_executable(robot_check_pose src/robot_check_pose.cpp)
target_link_libraries(robot_check_pose ${catkin_LIBRARIES})

add_executable(waypoint_set src/waypoint_set.cpp)
target_link_libraries(waypoint_set ${catkin_LIBRARIES})

add_executable(4goal_waypoint_send src/4goal_waypoint_send.cpp)
target_link_libraries(4goal_waypoint_send ${catkin_LIBRARIES})

add_executable(4goal_waypoint_send_2 src/4goal_waypoint_send_2.cpp)
target_link_libraries(4goal_waypoint_send_2 ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS 4goal_waypoint_send
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS 4goal_waypoint_send_2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS goal_waypoint_control_key
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)