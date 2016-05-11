cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_transform)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

add_executable(ssl_robot_transform src/ssl_robot_transform.cpp)
target_link_libraries(ssl_robot_transform ${catkin_LIBRARIES})
