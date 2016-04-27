cmake_minimum_required(VERSION 2.8.3)
project(basketball_maker_drawing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  basketball_msgs
  visualization_msgs
  nav_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES basketball_maker_drawing
  CATKIN_DEPENDS roscpp rospy visualization_msgs nav_msgs tf
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_rviz_debug src/basketball_draw_marker.cpp
 src/basketball_mark_srv.cpp
 )
target_link_libraries(robot_rviz_debug ${catkin_LIBRARIES})
add_dependencies(robot_rviz_debug basketball_msgs_gencpp)

