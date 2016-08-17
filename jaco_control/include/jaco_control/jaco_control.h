#ifndef JACO_CONTROL_H_
#define JACO_CONTROL_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <joint_state_server/GetAngularPosition.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco_kinematics_kdl/jaco_kinematics_kdl.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace ros;

class Kinematics;

class RobotArm
{
private:
  ServiceClient client_joints;
  Publisher velocity_pub, gripper_pub;
  Subscriber goal_sub, camera_sub;

  joint_state_server::GetAngularPosition srv_joints;

  string interface, description;
  int num_joints;
  geometry_msgs::Transform start_pos;
  vector<double> start_jnt;

public:
  Kinematics* kinematics;

  RobotArm(NodeHandle n, string name_vel_pub, string name_grip_pub)
  {
    n.param("robot_description", description, std::string());

    kinematics = new Kinematics(description,"jaco_link_base","jaco_link_hand");
    num_joints = kinematics->nj;

    velocity_pub = n.advertise<std_msgs::Float64MultiArray>(name_vel_pub,1);
    gripper_pub = n.advertise<std_msgs::Float64MultiArray>(name_grip_pub,1);
    client_joints = n.serviceClient<joint_state_server::GetAngularPosition>("get_angular_position");

    goal_sub = n.subscribe("/goal", 1 , &RobotArm::setCartesianGoal, this);
    //camera_sub = n.subscribe("/camera_goal", 1 , &RobotArm::cameraCallback, this);

    start_jnt = getJointState().position;
  }
 ~RobotArm()
 {
   free(kinematics);
 }

  //Set goal joint velocity
  void setJointVelocity(vector<float> joint_velocity);

  //Set a goal joint position
  void setJointPosition(vector<float> joint_goal);

  //Set a goal pose in Cartesian space
  void setCartesianGoal(geometry_msgs::Transform goal);

  //Set a goal gripper state
  void setGripper(vector<float> goal);

  //Call service that returns current joint state
  sensor_msgs::JointState getJointState();

  //Scale joint velocities
  vector<float> scaleJointVelocities(vector<float> vels);

};

#endif /* JACO_CONTROL_H_ */
