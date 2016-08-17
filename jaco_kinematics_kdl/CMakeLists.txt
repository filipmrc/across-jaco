cmake_minimum_required(VERSION 2.8.3)
project(jaco_kinematics_kdl)

find_package(catkin REQUIRED COMPONENTS
  joint_state_server
  roscpp
  wpi_jaco_wrapper
  actionlib
  tf_conversions
  hardware_interface
  controller_manager
)
find_package(kdl_parser REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(jaco_kinematics_kdl
  src/jaco_kinematics_kdl.cpp
)

target_link_libraries(jaco_kinematics_kdl
   ${catkin_LIBRARIES}
   ${kdl_parser_LIBRARIES}
 )
