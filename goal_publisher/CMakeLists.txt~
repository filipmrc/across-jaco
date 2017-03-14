cmake_minimum_required(VERSION 2.8.3)
project(goal_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  jaco_control
  jaco_kinematics_kdl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_publisher src/goal_publisher/goal_publisher.cc)

target_link_libraries(goal_publisher
   ${catkin_LIBRARIES}
   jaco_control
)

