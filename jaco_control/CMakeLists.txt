cmake_minimum_required(VERSION 2.8.3)
project(jaco_control)

find_package(catkin REQUIRED COMPONENTS
  joint_state_server
  roscpp
  wpi_jaco_wrapper
  actionlib
  tf_conversions
  hardware_interface
  controller_manager
  jaco_kinematics_kdl
)
find_package(kdl_parser REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(jaco_control src/jaco_control.cc)

target_link_libraries(jaco_control
   ${catkin_LIBRARIES}
   ${kdl_parser_LIBRARIES}
   jaco_kinematics_kdl
 )



