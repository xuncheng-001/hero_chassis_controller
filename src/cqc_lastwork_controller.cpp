//
// Created by xuncheng on 24-11-28.
//
#include <pluginlib/class_list_macros.hpp>
#include "hero_chassis_controller/cqc_lastwork_controller.h"
namespace cqc_lastwork_controller{
    bool CqcLastworkController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                     ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){

        front_left_joint = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint = effort_joint_interface->getHandle("right_back_wheel_joint");

        return true;
    }

    void CqcLastworkController::update(const ros::Time& time, const ros::Duration& period)
    {
        front_left_joint.setCommand(200);
        front_right_joint.setCommand(200);
        back_left_joint.setCommand(200);
        back_right_joint.setCommand(200);
    }
    PLUGINLIB_EXPORT_CLASS(cqc_lastwork_controller::CqcLastworkController,controller_interface::ControllerBase)
}