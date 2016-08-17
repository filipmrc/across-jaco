#include <jaco_control/jaco_control.h>

void RobotArm::setCartesianGoal(geometry_msgs::Transform goal)
{
  vector<double> q = getJointState().position;

  vector<float> vels = kinematics->getIKvel(kinematics->FK(), goal, q);

  setJointVelocity(vels);
}

sensor_msgs::JointState RobotArm::getJointState()
{
  sensor_msgs::JointState state;

  if (client_joints.call(srv_joints))
    {
      state = srv_joints.response.state;
    }
  else
    {
      ROS_ERROR("Failed to call service get_angular_position.");
    }
  return state;
}

void RobotArm::setJointVelocity(std::vector<float> joint_goal)
{
  std_msgs::Float64MultiArray joint;

  joint_goal.resize(num_joints);
  joint_goal = scaleJointVelocities(joint_goal);
  for (int i = 0; i<num_joints; i++)
    {
      joint.data.push_back(joint_goal[i]);
      if(abs(joint_goal[i]) < 0.0001){ joint_goal[i] = 0.00;}
    }
  velocity_pub.publish(joint);
}

void RobotArm::setJointPosition(vector<float> joint_goal)
{
  vector<float> e(num_joints); vector<double> q = getJointState().position;
  q.resize(num_joints);
  for (int var = 0; var < num_joints; ++var) {
      e[var] = joint_goal[var] - q[var];
  }
  setJointVelocity(e);
}

vector<float> RobotArm::scaleJointVelocities(vector<float> vels)
{
  float max_vel = 0.0, max_acc = 0.0;

  for (unsigned int i = 0; i < num_joints; i++)
  {
    max_vel = max(abs(vels[i]), max_vel);
  }

  double joint_velocity_limit = 2.50;
  double scale_vel = 1.0;
  if (max_vel > joint_velocity_limit)
    {
      double scale_vel = joint_velocity_limit / max_vel;
      for (unsigned int i = 0; i < num_joints; ++i)
	{
	  vels[i] *= scale_vel;
	}
    }

  vector<double> vels_old(num_joints);
  vels_old = getJointState().velocity;

  for (unsigned int i = 0; i < num_joints; i++)
    {
      max_acc = max(abs(vels[i]- float(vels_old[i])), max_acc);
    }

  double joint_acceleration_limit = 0.25;
  double scale_acc = 1.0;
  if (max_acc > joint_acceleration_limit)
    {
      //ROS_INFO_STREAM("Limiting acceleration!");
      scale_acc = joint_acceleration_limit / max_acc;
      for (unsigned i = 0; i < num_joints; ++i)
	{
	  vels[i] *= scale_acc;
	}
      kinematics->ikvelsolver->setLambda(0.02);
    }
  else
    {
      kinematics->ikvelsolver->setLambda(0.00);
    }
  return vels;
}
