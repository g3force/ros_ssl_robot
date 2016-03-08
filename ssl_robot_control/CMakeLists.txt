cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(vel_xyw2motorCtrl src/vel_xyw2motorCtrl.cpp)
target_link_libraries(vel_xyw2motorCtrl ${catkin_LIBRARIES})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
