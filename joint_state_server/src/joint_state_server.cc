#include <ros/ros.h>
#include <joint_state_server/joint_state_server.h>


JointStateServer::JointStateServer()
{
  service = n.advertiseService("get_angular_position", &JointStateServer::getAngularPosition,this);
  sub = n.subscribe("/joint_states", 1, &JointStateServer::jointStatesCallback, this);
  ROS_INFO("angular position server ready");
  //pos.resize(6);
  //vel.resize(6);
}

void JointStateServer::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state_)
{
  for (int var = 0; var < 6; var++)
    {
      //pos[var] = state->position[var];
      //vel[var] = state->velocity[var];
      state = *state_;
    }
}

bool JointStateServer::getAngularPosition(joint_state_server::GetAngularPosition::Request &req , joint_state_server::GetAngularPosition::Response &res)
{
/*  res.pos.resize(6);
  for (int var = 0; var < 6; var++)
    {
      res.pos[var]= pos[var];
    }
  res.vel.resize(6);
  for (int var = 0; var < 6; var++)
    {
      res.pos[var]= pos[var];
    }*/
  res.state = state;
return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_server");
  JointStateServer server;
  ros::spin();

  return 0;
}
