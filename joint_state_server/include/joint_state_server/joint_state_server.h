/*
 * joint_state_server.h
 *
 *  Created on: Oct 20, 2015
 *      Author: marki
 */
#ifndef JOINT_STATE_SERVER_H_
#define JOINT_STATE_SERVER_H_

#include <joint_state_server/GetAngularPosition.h>
#include <sensor_msgs/JointState.h>

class JointStateServer
{
private:
    //std::vector<double> pos, vel;
    ros::NodeHandle n;
    ros::ServiceServer service;
    ros::Subscriber sub;
    sensor_msgs::JointState state;
public:
	JointStateServer();
	void jointStatesCallback(const sensor_msgs::JointStateConstPtr &state);
	bool getAngularPosition(joint_state_server::GetAngularPosition::Request &req , joint_state_server::GetAngularPosition::Response &res);
};
#endif /* JOINT_STATE_SERVER_H_ */
