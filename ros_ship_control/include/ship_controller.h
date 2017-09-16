#ifndef SHIP_CONTROLLER_H_INCLUDED
#define SHIP_CONTROLLER_H_INCLUDED

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

namespace ship_controller
{
  class ShipController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    ShipController();

    virtual bool init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& root_nh,ros::NodeHandle& controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time& time);

  private:
    hardware_interface::JointHandle left_motor_joint;
    hardware_interface::JointHandle right_motor_joint;
    ros::Subscriber sub_command;
  };
}

#endif
