//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>


namespace hero_chassis_controller{
bool CqcLastWorkController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  return true;
}

void CqcLastWorkController::update(const ros::Time& time, const ros::Duration& period)
{

  front_left_joint_.setCommand(2.0 );

}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::CqcLastWorkController,  controller_interface::ControllerBase)
}  // namespace simple_chassis_controller
