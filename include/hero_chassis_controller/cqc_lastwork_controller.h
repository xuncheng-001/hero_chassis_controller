//
// Created by xuncheng on 24-11-28.
//

#ifndef RM_DESCRIPTION_CQC_LASTWORK_CONTROLLER_H
#define RM_DESCRIPTION_CQC_LASTWORK_CONTROLLER_H

#include <controller_interface/controller_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace cqc_lastwork_controller
{
class CqcLastworkController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
//继承controller_interface::Controller，并且是和hardware_interface::EffortJointInterface关联，也就是控制力矩的
{
public:
    CqcLastworkController() = default;
    ~CqcLastworkController()override = default;

    bool init(hardware_interface::EffortJonitInterface* effort_joint_interface, ros::NodeHandle& root_nh,ros::Nodehandle& controller_nh )override;

    void update(const ros::Time$ time ,const ros::Duration& period)override;

    hardware_interface::JointHandle front_left_joint, front_right_joint, back_left_joint, back_right_joint;
};
}





#endif //RM_DESCRIPTION_CQC_LASTWORK_CONTROLLER_H
