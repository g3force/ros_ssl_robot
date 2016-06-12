cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_vision)

include(FindProtobuf)
find_package( Protobuf REQUIRED )

FILE(GLOB ProtoFiles "proto/*.proto")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ProtoFiles})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

add_executable(ssl_robot_vision src/ssl_robot_vision.cpp ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(ssl_robot_vision ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
