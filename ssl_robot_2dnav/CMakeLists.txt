cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_2dnav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  move_base
  move_base_msgs
  actionlib
)

catkin_package(
  CATKIN_DEPENDS roscpp move_base move_base_msgs actionlib
)

add_executable(ssl_robot_move_to src/ssl_robot_move_to.cpp)
target_link_libraries(ssl_robot_move_to ${catkin_LIBRARIES})
