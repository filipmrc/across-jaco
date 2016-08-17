#include <jaco_kinematics_kdl/jaco_kinematics_kdl.h>

 Kinematics::Kinematics(string description, string base, string end_effector)
 {
   Tree my_tree;

   //parse the urdf and create a kdl::tree
   if (!kdl_parser::treeFromString(description, my_tree))
   {
     ROS_ERROR("Failed to construct kdl tree");
     return;
   }

   //get chain from tree
   my_tree.getChain(base,end_effector,chain);
   nj = chain.getNrOfJoints();

   //initialize solvers
   fksolver = new ChainFkSolverPos_recursive(chain);
   ikvelsolver = new ChainIkSolverVel_wdls(chain);
 }

 Kinematics::~Kinematics()
 {
   free(ikvelsolver);
   free(fksolver);
 }

 geometry_msgs::Transform Kinematics::FK()
 {
   StampedTransform t;
   geometry_msgs::TransformStamped msg;
   try{
     listener.lookupTransform("/root", "/jaco_link_hand",
                              Time(0), t);
     transformStampedTFToMsg(t,msg);
     return msg.transform;
   }
   catch (TransformException &ex){
     ROS_ERROR("%s",ex.what());
     Duration(1.0).sleep();
   }
 }

vector<float> Kinematics::getIKvel(geometry_msgs::Transform current , geometry_msgs::Transform goal, vector<double> jpos_)
  {
    // Create joint array
    JntArray jpos(nj),jvel(nj);

    //delta.rot.Zero();
    jpos_.resize(nj);
    // Assign some values to the joint positions
    for(int i=0;i<nj;i++){
        jpos(i)= jpos_[i];
    }

    Twist e = getError(goal,current);

    vector<float> vel_res(nj);
    ikvelsolver->setLambda(0.02);

    if(ikvelsolver->CartToJnt(jpos, e, jvel)<0)
    {
      for(unsigned int i = 0; i < nj; i++)
      {
        jvel(i) = 0.0;
      }
    }

    for (unsigned int i = 0; i < nj; i++)
    {
      vel_res[i]= jvel(i);
      if (!isfinite(vel_res[i]))
      {
        ROS_ERROR_THROTTLE(1.0, "Target joint velocity (%d) is not finite : %f", i, vel_res[i]);
        vel_res[i] = 1.0;
      }
    }
    return vel_res;
  }

Twist Kinematics::getError(geometry_msgs::Transform goal_ , geometry_msgs::Transform current_)
{
  Frame goal, current;

  transformMsgToKDL(goal_, goal);
  transformMsgToKDL(current_, current);

  Twist error;
  error.vel = goal.p - current.p;
  error.rot = 0.5*(current.M.UnitX()*(goal.M.UnitX()) +
                   current.M.UnitY()*(goal.M.UnitY()) +
	           current.M.UnitZ()*(goal.M.UnitZ()));

  double norm_vel = error.vel.Norm(), norm_rot = error.rot.Norm();

  if(norm_vel < 0.00001){error.vel = KDL::Vector::Zero();}
  if(norm_rot < 0.00001){error.rot = KDL::Vector::Zero();}
  if(goal.p.x() == 234476){error.vel = KDL::Vector::Zero();}

  return error;
}

