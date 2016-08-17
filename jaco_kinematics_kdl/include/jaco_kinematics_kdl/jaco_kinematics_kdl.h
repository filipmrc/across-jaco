/*
 * jaco_kinematics.h
 *
 *  Created on: Oct 30, 2015
 *      Author: Filip Marić
 */

#ifndef JACO_KINEMATICS_H_
#define JACO_KINEMATICS_H_

#include <cmath>
#include <tf/tf.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_listener.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/kdl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <std_msgs/Float64.h>

//Link lengths and offsets
#define D1 .2755
#define D2 .4100
#define D3 .2073
#define D4 .0743
#define D5 .0743
#define D6 .1687
#define E2 .0098
#define PI 3.14159

using namespace std;
using namespace ros;
using namespace KDL;
using namespace tf;

class Kinematics
{
private:
  //KDL chain and solvers
  KDL::Chain chain;
  KDL::ChainFkSolverPos_recursive* fksolver;
  tf::TransformListener listener;

public:

  KDL::ChainIkSolverVel_wdls* ikvelsolver;

  Kinematics (string description, string base, string end_effector);

  ~Kinematics();

  std::vector<float> getIKvel(geometry_msgs::Transform goal , geometry_msgs::Transform current, std::vector<double> jpos_);

  Twist getError(geometry_msgs::Transform goal_ , geometry_msgs::Transform current_);

  geometry_msgs::Transform FK();

  unsigned int nj;
};

#endif /* JACO_KINEMATICS_H_ */
