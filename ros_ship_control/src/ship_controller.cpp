//headers in this package
#include <ship_controller.h>

//headers for ROS
#include <geometry_msgs/Twist.h>

namespace ship_controller
{
  ShipController::ShipController():command_struct()
  {

  }

  bool ShipController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh)
  {
    std::string left_motor_joint_name,right_motor_joint_name;
    controller_nh.getParam("left_motor",left_motor_joint_name);
    controller_nh.getParam("right_motor",right_motor_joint_name);
    controller_nh.getParam("linear/max_velocity",max_linear_velocity);
    controller_nh.getParam("linear/min_velocity",min_linear_velocity);
    controller_nh.getParam("angular/max_velocity",max_angular_velocity);
    controller_nh.getParam("angular/min_velocity",min_angular_velocity);
    sub_command = controller_nh.subscribe("cmd_vel", 1, &ShipController::cmdVelCallback, this);
  }

  void ShipController::starting(const ros::Time& time)
  {

  }

  void ShipController::update(const ros::Time& time, const ros::Duration& period)
  {

  }

  void ShipController::stopping(const ros::Time& time)
  {

  }

  void ShipController::cmdVelCallback(const geometry_msgs::Twist& msg)
  {
    if (isRunning())
    {
      command_struct.ang   = msg.angular.z;
      command_struct.lin   = msg.linear.x;
      command_struct.stamp = ros::Time::now();
      command.writeFromNonRT(command_struct);
      ROS_INFO_STREAM("Added values to command. " << "Ang: "   << command_struct.ang << ", " << "Lin: "   << command_struct.lin << ", " << "Stamp: " << command_struct.stamp);
    }
    else
    {
      ROS_ERROR("Can't accept new commands. Controller is not running.");
    }
  }
}
