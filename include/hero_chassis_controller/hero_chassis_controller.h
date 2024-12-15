

#include <regex>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <yaml-cpp/node/convert.h>
#include <control_toolbox/pid.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>




namespace hero_chassis_controller{

    hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
    control_toolbox::Pid front_left_pid_, front_right_pid_, back_left_pid_, back_right_pid_;
    ros::Publisher controller_state_publisher_;
    double wheel_radius_=0.07625;
   double wheel_base_=0.4;
    double wheel_track_=0.4;
    double R=0.2828;


    double target_fl_speed;
    double target_fr_speed;
    double target_bl_speed;
    double target_br_speed;

class PidGains {
    public:
    double p;
    double i;
    double d;
    double i_clamp;
    double i_antiwindup;
};
class CqcLastWorkController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  CqcLastWorkController() =default;
  ~CqcLastWorkController() override = default;
  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;





    void setGains(control_toolbox::Pid& pid_controller_,PidGains& pid_gains) {
        pid_controller_.setGains(pid_gains.p,pid_gains.i,pid_gains.d,pid_gains.i_clamp,0.0,pid_gains.i_antiwindup);

    }
    PidGains pid_front_left_gains,pid_front_right_gains,pid_back_left_gains,pid_back_right_gains;
void Get_Pid(ros::NodeHandle& node_handle,PidGains& gains,const std::string& pid_name) {
    node_handle.getParam("/"+pid_name + "/p",gains.p);
    node_handle.getParam("/"+pid_name + "/i",gains.i);
    node_handle.getParam("/"+pid_name + "/d",gains.d);
    node_handle.getParam("/"+pid_name + "/i_clamp",gains.i_clamp);
}
    void cmdSub(const geometry_msgs::Twist& cmd_vel) {

    if (use_global_frame_) {
        tf::StampedTransform transform;
        tf_listener_.lookupTransform("base_link", "odom", ros::Time(0), transform);


        tf::Matrix3x3 m(transform.getRotation());
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        geometry_msgs::Twist transformed_cmd_vel;
        transformed_cmd_vel.linear.x = cmd_vel.linear.x * cos(yaw) - cmd_vel.linear.y * sin(yaw);
        transformed_cmd_vel.linear.y = cmd_vel.linear.x * sin(yaw) + cmd_vel.linear.y * cos(yaw);
        transformed_cmd_vel.angular.z = cmd_vel.angular.z;

        target_fl_speed = (transformed_cmd_vel.linear.x - transformed_cmd_vel.linear.y - transformed_cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2) / (2.0 * wheel_radius_);
        target_fr_speed = (transformed_cmd_vel.linear.x + transformed_cmd_vel.linear.y + transformed_cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2) / (2.0 * wheel_radius_);
        target_bl_speed = (transformed_cmd_vel.linear.x - transformed_cmd_vel.linear.y + transformed_cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2) / (2.0 * wheel_radius_);
        target_br_speed = (transformed_cmd_vel.linear.x + transformed_cmd_vel.linear.y - transformed_cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2) / (2.0 * wheel_radius_);
    } else {
        target_fl_speed = (cmd_vel.linear.x + cmd_vel.linear.y - cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2)  /(2.0  * wheel_radius_);
        target_fr_speed = (cmd_vel.linear.x - cmd_vel.linear.y + cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2)  /(2.0  * wheel_radius_);
        target_bl_speed = (cmd_vel.linear.x - cmd_vel.linear.y - cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2) /(2.0  * wheel_radius_);
        target_br_speed = (cmd_vel.linear.x + cmd_vel.linear.y + cmd_vel.angular.z * (wheel_base_ + wheel_track_)/2)/(2.0  * wheel_radius_);
    }

}

  void setCommandCallback(const std_msgs::Float64 &msg) {
      command_.data =msg.data;
  }

private:
  int state_{};
  ros::Time last_change_;
    ros::Duration period_;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_state_;
    std_msgs::Float64 command_;
    bool use_global_frame_=false;
   ros::Subscriber cmd_vel_sub_;


    tf::TransformBroadcaster *tf_broadcaster_;
    ros::NodeHandle nh_;
    ros::Subscriber wheel_speed_sub_;
    ros::Time last;
    ros::Publisher odom_pub_;
    double x_,y_,z_;
    double last_x_=0,last_y_=0;
    double last_z_=0;
    double dt;

    tf::TransformListener tf_listener_;
    void publishOdom(const ros::Time& now) {

        dt =now.toSec()-last.toSec();

        double v_f_l = wheel_radius_*front_left_joint_.getVelocity();
        double v_f_r = wheel_radius_*front_right_joint_.getVelocity();
        double v_b_l = wheel_radius_*back_left_joint_.getVelocity();
        double v_b_r = wheel_radius_*back_right_joint_.getVelocity();

        double v_x = (v_f_l+v_f_r+v_b_l+v_b_r)/4;
        double v_y = (-v_f_l+v_f_r-v_b_l+v_b_r)/4;
        double omega =(v_f_l+v_f_r-v_b_l-v_b_r)/(4*(wheel_base_+wheel_track_)/2);

        double delta_x = (v_x * cos(z_) - v_y * sin(z_)) * dt;
        double delta_y = (v_x * sin(z_) + v_y * cos(z_)) * dt;
        double delta_z = omega * dt;

        x_ += delta_x;
        y_ += delta_y;
        z_ += delta_z;


        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_ ;
        odom.pose.pose.position.y = y_ ;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(z_);

        odom.twist.twist.linear.x = (x_-last_x_)/dt;
        odom.twist.twist.linear.y = (y_-last_y_)/dt;
        odom.twist.twist.angular.z = (z_-last_z_)/dt;

        odom_pub_.publish(odom);

        last_x_=x_;
        last_y_=y_;
        last_z_=z_;
        last = now;

        tf2_ros::TransformBroadcaster odom_broadcaster;

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";


        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(z_);


      odom_broadcaster.sendTransform(odom_trans);



    }


};
}
