//
// Created by xuncheng on 24-11-28.
//
#include <pluginlib/class_list_macros.hpp>
#include "hero_chassis_controller/hero_chassis_controller.h"
namespace hero_chassis_controller
{
    bool CqcLastWorkController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){

        front_left_joint = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint = effort_joint_interface->getHandle("right_back_wheel_joint");

        return true;
    }




    void CqcLastWorkController::update(const ros::Time& time, const ros::Duration& period)
    {
    front_left_joint.setCommand(2.0);
    }


   PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::CqcLastWorkController,controller_interface::ControllerBase)
}