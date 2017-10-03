//headers in this package
#include <driving_force_controller.h>

//headers for controller_interface
#include <controller_interface/controller.h>

//headers for pluginlib
#include <pluginlib/class_list_macros.h>

namespace driving_force_controller
{
  DrivingForceController::DrivingForceController()
  {

  }

  DrivingForceController::~DrivingForceController()
  {

  }

  bool DrivingForceController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh)
  {

  }
  void DrivingForceController::starting(const ros::Time& time)
  {

  }
  void DrivingForceController::update(const ros::Time& time, const ros::Duration& period)
  {

  }
  void DrivingForceController::stopping(const ros::Time& time)
  {

  }
}

PLUGINLIB_EXPORT_CLASS(driving_force_controller::DrivingForceController, controller_interface::ControllerBase);
