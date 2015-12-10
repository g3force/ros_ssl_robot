cmake_minimum_required(VERSION 2.8.3)
project(ssl_robot_description)

find_package(catkin REQUIRED)

#~ find_package(xacro REQUIRED)
#~ file(GLOB xacro_files ${CMAKE_CURRENT_SOURCE_DIR}/urdf/ssl_robot.xacro)
#~ 
#~ foreach(it ${xacro_files})
  #~ # remove .xacro extension
  #~ string(REGEX MATCH "(.*)[.]xacro$" unused ${it})
  #~ set(output_filename ${CMAKE_MATCH_1}.urdf)
#~ 
  #~ # create a rule to generate ${output_filename} from {it}
  #~ xacro_add_xacro_file(${it} ${output_filename})
#~ 
  #~ list(APPEND world_files ${output_filename})
#~ endforeach(it)
#~ 
#~ # add an abstract target to actually trigger the builds
#~ add_custom_target(media_files ALL DEPENDS ${world_files})


find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
