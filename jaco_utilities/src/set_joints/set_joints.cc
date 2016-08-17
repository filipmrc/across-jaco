#include <jaco_control/jaco_control.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_joint_set");
  ros::NodeHandle n;
  RobotArm arm(n,"/jaco_arm_controller/command","/jaco_gripper_controller/command");
  ros::Rate r(10);
  sleep(1);

  std::vector<float> a_goal(6); std::vector<double> q(6);
  a_goal[0] = PI; a_goal[1] = PI*2.2/2; a_goal[2] = PI/2; a_goal[3] = 0; a_goal[4] = PI; a_goal[5] = PI/2;

  while(ros::ok())
  {
    arm.setJointPosition(a_goal);
    q = arm.getJointState().position;
    ros::spinOnce();
    r.sleep();
  }
}
