//headers in this package
#include <driving_force_controller.h>

//headers for controller_interface
#include <controller_interface/controller.h>

//headers for pluginlib
#include <pluginlib/class_list_macros.h>

//headers for realtime_tools
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

//headers for boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

namespace driving_force_controller
{
  DrivingForceController::DrivingForceController()
  {

  }

  DrivingForceController::~DrivingForceController()
  {

  }

  void DrivingForceController::drivingForceCallback(const std_msgs::Float32& msg)
  {
    if (isRunning())
    {
      driving_force_struct.value = msg.data;
      driving_force.writeFromNonRT(driving_force_struct);
    }
    else
    {
      ROS_ERROR("Can't accept new commands. Controller is not running.");
    }
  }

  void DrivingForceController::twistCallback(const geometry_msgs::Twist& msg)
  {
    if (isRunning())
    {
      twist_struct.ang_x = msg.angular.x;
      twist_struct.ang_y = msg.angular.y;
      twist_struct.ang_z = msg.angular.z;
      twist_struct.lin_x = msg.linear.x;
      twist_struct.lin_y = msg.linear.y;
      twist_struct.lin_z = msg.linear.z;
      twist_struct.stamp = ros::Time::now();
      twist.writeFromNonRT(twist_struct);
    }
    else
    {
      ROS_ERROR("Can't accept new commands. Controller is not running.");
    }
  }

  bool DrivingForceController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh)
  {
    controller_nh.getParam("twist_topic",twist_topic);
    controller_nh.getParam("motor_command_topic",motor_command_topic);
    controller_nh.getParam("driving_force_command_topic",driving_force_command_topic);
    motor_command_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(controller_nh, motor_command_topic, 1));
    sub_twist = controller_nh.subscribe(twist_topic, 1, &DrivingForceController::twistCallback, this);
    driving_force_sub = controller_nh.subscribe(driving_force_command_topic, 1, &DrivingForceController::drivingForceCallback, this);
  }

  void DrivingForceController::starting(const ros::Time& time)
  {

  }

  void DrivingForceController::update(const ros::Time& time, const ros::Duration& period)
  {
    if(motor_command_publisher && motor_command_publisher->trylock())
    {

    }
  }
  void DrivingForceController::stopping(const ros::Time& time)
  {

  }
}

PLUGINLIB_EXPORT_CLASS(driving_force_controller::DrivingForceController, controller_interface::ControllerBase);
