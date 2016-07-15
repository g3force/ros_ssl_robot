cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  gazebo_msgs
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs gazebo_msgs nav_msgs tf
)

add_library(AngleMath src/AngleMath.cpp)
target_link_libraries(AngleMath ${caktin_LIBRARIES})

add_executable(ssl_robot_feedback_gaz src/ssl_robot_feedback_gaz.cpp)
target_link_libraries(ssl_robot_feedback_gaz ${catkin_LIBRARIES} AngleMath)

add_executable(ssl_robot_feedback_gaz_p3d src/ssl_robot_feedback_gaz_p3d.cpp)
target_link_libraries(ssl_robot_feedback_gaz_p3d ${catkin_LIBRARIES} AngleMath)

add_executable(pose_add_cov src/pose_add_cov.cpp)
target_link_libraries(pose_add_cov ${catkin_LIBRARIES})
