
#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace hero_chassis_controller{


    double cmd_front_left_wheel_,cmd_front_right_wheel_,cmd_back_left_wheel_,cmd_back_right_wheel_;


bool CqcLastWorkController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  root_nh.getParam("use_global_frame", use_global_frame_);


  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");


  front_left_pid_.init(root_nh, "front_left_wheel_pid");
  front_right_pid_.init(root_nh, "front_right_wheel_pid");
  back_left_pid_.init(root_nh, "back_left_wheel_pid");
  back_right_pid_.init(root_nh, "back_right_wheel_pid");

  Get_Pid(controller_nh,pid_front_left_gains,"front_left_wheel_pid");
  setGains(front_left_pid_,pid_front_left_gains);

  Get_Pid(controller_nh,pid_front_right_gains,"front_right_wheel_pid");
  setGains(front_right_pid_,pid_front_right_gains);
  Get_Pid(controller_nh,pid_back_left_gains,"back_left_wheel_pid");
  setGains(back_left_pid_,pid_back_left_gains);
  Get_Pid(controller_nh,pid_back_right_gains,"back_right_wheel_pid");
  setGains(back_right_pid_,pid_back_right_gains);

    sub_command_ =root_nh.subscribe("command",1,&CqcLastWorkController::setCommandCallback,this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom",10);


 cmd_vel_sub_ = nh_.subscribe("/cmd_vel",10,&CqcLastWorkController::cmdSub,this);
  return true;
}



  double error1 =0;
  double error2 =0;
  double error3 =0;
  double error4 =0;




void CqcLastWorkController::update(const ros::Time& time, const ros::Duration& period) {







  publishOdom(time);




    error1 = target_fl_speed -wheel_radius_*front_left_joint_.getVelocity();
   cmd_front_left_wheel_ = front_left_pid_.computeCommand(error1,ros::Duration(0.01));
  front_left_joint_.setCommand(cmd_front_left_wheel_);
    error2 = target_fr_speed - wheel_radius_*front_right_joint_.getVelocity();
    cmd_front_right_wheel_ = front_right_pid_.computeCommand(error2,ros::Duration(0.01));
    front_right_joint_.setCommand(cmd_front_right_wheel_);

    error3 = target_bl_speed - wheel_radius_*back_left_joint_.getVelocity();
    cmd_back_left_wheel_ = back_left_pid_.computeCommand(error3,ros::Duration(0.01));
    back_left_joint_.setCommand(cmd_back_left_wheel_);

    error4 = target_br_speed - wheel_radius_*back_right_joint_.getVelocity();
    cmd_back_right_wheel_ = back_right_pid_.computeCommand(error4,ros::Duration(0.01));
    back_right_joint_.setCommand(cmd_back_right_wheel_);

  }


  PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::CqcLastWorkController,  controller_interface::ControllerBase)
}
