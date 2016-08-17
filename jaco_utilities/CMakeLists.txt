cmake_minimum_required(VERSION 2.8.3)
project(jaco_utilities)

find_package(catkin REQUIRED COMPONENTS
  joint_state_server
  roscpp
  wpi_jaco_wrapper
  actionlib
  tf_conversions
  hardware_interface
  controller_manager
  jaco_control
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
 
add_executable(set_joints src/set_joints/set_joints.cc)
 
target_link_libraries(set_joints
   ${catkin_LIBRARIES}
   ${kdl_parser_LIBRARIES}
   jaco_kinematics_kdl
   jaco_control
)

