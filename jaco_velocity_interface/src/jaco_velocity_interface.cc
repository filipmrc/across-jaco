#include <jaco_velocity_interface/jaco_velocity_interface.h>
class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot()
 {
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    // ROS_INFO("Trying to initialize JACO API...");
    while ( InitAPI() != NO_ERROR )
    {
      ROS_ERROR("Could not initialize Kinova API. Is the arm connected?");
      ROS_INFO("Retrying in 5 seconds..");
      ros::Duration(5.0).sleep();
    }

    ros::Duration(1.0).sleep();
    StartControlAPI();
    ros::Duration(3.0).sleep();
    StopControlAPI();

    // Initialize arm
    bool home_arm = true;
    MoveHome();
    InitFingers();
    SetFrameType(0); //set end effector to move with respect to the fixed frame

    ROS_INFO("Arm initialized.");

    StartControlAPI();
    SetAngularControl();
    controlType = ANGULAR_CONTROL;
    eStopEnabled = false;
  }

  void initInterface()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_1("jaco_joint_1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("jaco_joint_2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_2);

    hardware_interface::JointStateHandle state_handle_3("jaco_joint_3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_3);

    hardware_interface::JointStateHandle state_handle_4("jaco_joint_4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_4);

    hardware_interface::JointStateHandle state_handle_5("jaco_joint_5", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_5);

    hardware_interface::JointStateHandle state_handle_6("jaco_joint_6", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_6);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("jaco_joint_1"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("jaco_joint_2"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_2);

    hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("jaco_joint_3"), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_3);

    hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("jaco_joint_4"), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_4);

    hardware_interface::JointHandle vel_handle_5(jnt_state_interface.getHandle("jaco_joint_5"), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_5);

    hardware_interface::JointHandle vel_handle_6(jnt_state_interface.getHandle("jaco_joint_6"), &cmd[5]);
    jnt_vel_interface.registerHandle(vel_handle_6);

    registerInterface(&jnt_vel_interface);
  }

  void getJointStates()
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    AngularPosition force_data;
    GetAngularForce(force_data);
    eff[0] = force_data.Actuators.Actuator1;
    eff[1] = force_data.Actuators.Actuator2;
    eff[2] = force_data.Actuators.Actuator3;
    eff[3] = force_data.Actuators.Actuator4;
    eff[4] = force_data.Actuators.Actuator5;
    eff[5] = force_data.Actuators.Actuator6;

    AngularPosition velocity_data;
    GetAngularVelocity(velocity_data);
    vel[0] = velocity_data.Actuators.Actuator1 * DEG_TO_RAD;
    vel[1] = velocity_data.Actuators.Actuator2 * DEG_TO_RAD;
    vel[2] = velocity_data.Actuators.Actuator3 * DEG_TO_RAD;
    vel[3] = velocity_data.Actuators.Actuator4 * DEG_TO_RAD;
    vel[4] = velocity_data.Actuators.Actuator5 * DEG_TO_RAD;
    vel[5] = velocity_data.Actuators.Actuator6 * DEG_TO_RAD;

    AngularPosition position_data;
    GetAngularPosition(position_data);
    pos[0] = simplify_angle(position_data.Actuators.Actuator1 * DEG_TO_RAD);
    pos[1] = position_data.Actuators.Actuator2 * DEG_TO_RAD;
    pos[2] = position_data.Actuators.Actuator3 * DEG_TO_RAD;
    pos[3] = simplify_angle(position_data.Actuators.Actuator4 * DEG_TO_RAD);
    pos[4] = simplify_angle(position_data.Actuators.Actuator5 * DEG_TO_RAD);
    pos[5] = simplify_angle(position_data.Actuators.Actuator6 * DEG_TO_RAD);

  }

  void setJointVelocities(ros::Duration period)
  {
    if (eStopEnabled)
      return;

//    jnt_limits_interface.enforceLimits(period);
    //take control of the arm
    {
      boost::recursive_mutex::scoped_lock lock(api_mutex);
      EraseAllTrajectories();

      if (controlType != ANGULAR_CONTROL)
      {
        SetAngularControl();
        controlType = ANGULAR_CONTROL;
      }
    }

    TrajectoryPoint jacoPoint;
    jacoPoint.InitStruct();

    jacoPoint.Position.Type = ANGULAR_VELOCITY;
    jacoPoint.Position.Actuators.Actuator1 = cmd[0] * RAD_TO_DEG;
    jacoPoint.Position.Actuators.Actuator2 = cmd[1] * RAD_TO_DEG;
    jacoPoint.Position.Actuators.Actuator3 = cmd[2] * RAD_TO_DEG;
    jacoPoint.Position.Actuators.Actuator4 = cmd[3] * RAD_TO_DEG;
    jacoPoint.Position.Actuators.Actuator5 = cmd[4] * RAD_TO_DEG;
    jacoPoint.Position.Actuators.Actuator6 = cmd[5] * RAD_TO_DEG;
    jacoPoint.Position.HandMode = HAND_NOMOVEMENT;

    boost::recursive_mutex::scoped_lock lock(api_mutex);

    //send the command repeatedly for ~1/60th of a second
    //(this is sometimes necessary for velocity commands to work correctly)
    ros::Rate rate(600);
    for (int i = 0; i < 10; i++)
    {
      SendBasicTrajectory(jacoPoint);
      rate.sleep();
    }

  }

  static inline double simplify_angle(double angle)
  {
    double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
    double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
    double current_rev;
    if (fabs(angle - previous_rev) < fabs(angle - next_rev))
      return angle - previous_rev;
    return angle - next_rev;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_jnt_vel_int");
  ros::NodeHandle nh;

  MyRobot robot;
  robot.initInterface();

  ros::CallbackQueue my_callback_queue;
  nh.setCallbackQueue(&my_callback_queue);

  ros::AsyncSpinner spinner(0, &my_callback_queue);
  ros::Rate r(50);

  spinner.start();

  controller_manager::ControllerManager cm(&robot,nh);

  while(ros::ok())
  {
    robot.getJointStates();
    cm.update(ros::Time::now(), ros::Duration(r));
    robot.setJointVelocities(ros::Duration(r));
    ros::spinOnce();
    r.sleep();
  }
}
