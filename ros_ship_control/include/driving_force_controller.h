#ifndef DRIVING_FORCE_CONTROLLER_H_INCLUDED
#define DRIVING_FORCE_CONTROLLER_H_INCLUDED

//headers for controller_interface
#include <controller_interface/controller.h>


//headers for fardware_interface
#include <hardware_interface/joint_command_interface.h>

namespace driving_force_controller
{
  class DrivingForceController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    DrivingForceController();
    ~DrivingForceController();
  private:
    virtual bool init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time& time);
  };
}
#endif
