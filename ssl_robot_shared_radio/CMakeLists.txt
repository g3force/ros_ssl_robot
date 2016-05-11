cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_shared_radio)

include(FindProtobuf)
find_package( Protobuf REQUIRED )

FILE(GLOB ProtoFiles "proto/*.proto")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ProtoFiles})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

add_executable(vel_xyw2SharedRadio ${PROTO_SRCS} ${PROTO_HDRS} src/vel_xyw2SharedRadio.cpp)
target_link_libraries(vel_xyw2SharedRadio ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_executable(sharedRadio2vel_xyw ${PROTO_SRCS} ${PROTO_HDRS} src/sharedRadio2vel_xyw.cpp)
target_link_libraries(sharedRadio2vel_xyw ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})
