#include <ros/ros.h>
#include <jaco_control/jaco_control.h>

int state = 1;
double delta_x = 0, delta_y = 0, delta_z = 0;
geometry_msgs::Transform goal;

/*void cameraCallback(geometry_msgs::Transform camera_goal)
{
  geometry_msgs::Transform goal; KDL::Frame temp;

  double yaw = -PI/4 + camera_goal.rotation.y*PI/2;
  KDL::Rotation rot;

  temp = kinematics->getFK(getJointState().position);
  rot.DoRotY(yaw);
  temp.M = start_pos_.M*rot;
  tf::transformKDLToMsg(temp , goal);

  goal.translation.x = start_pos.translation.x + camera_goal.translation.x;
  goal.translation.y = start_pos.translation.y + camera_goal.translation.y;
  goal.translation.z = start_pos.translation.z + camera_goal.translation.z;

  setCartesianGoal(goal);
}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher");

  ros::NodeHandle n;
  ros::Rate r(50);

  RobotArm arm(n, "/jaco_arm_controller/command","/jaco_gripper_controller/command");
  ROS_INFO_STREAM("Control node successfully initialized!");

  ros::Publisher goal_pub = n.advertise<geometry_msgs::Transform>("/goal",1);

  goal.translation.x = 0.388825;
  goal.translation.y = -0.0889062;
  goal.translation.z = 0.3;
  goal.rotation.w = 0.514286;
  goal.rotation.x = -0.676797;
  goal.rotation.y = -0.279248;
  goal.rotation.z = -0.446629;

  double z_start = 0.3 , x_start = 0.4, y_start = -0.1;
  unsigned int i = 0;

  while(ros::ok())
    {
      if(i<=60){
	  goal.translation.x = x_start + i*0.07/60;
	  goal.translation.y = y_start + i*0.07/60;
      }else if(i<=120){
	  goal.translation.z = z_start + (i-60)*0.05/60;
      }else if(i<=180){
	  goal.translation.x = x_start + 0.07 - (i-120)*0.07/60;
	  goal.translation.y = y_start + 0.07 - (i-120)*0.07/60;
      }else if(i<=240){
	  goal.translation.z = z_start + 0.05 - (i-180)*0.05/60;
      }

      goal_pub.publish(goal);

      i == 240 ? i = 0 : i++;
      ros::spinOnce();
      r.sleep();
    }
}
