//headers in this package
#include <ship_controller.h>

namespace ship_controller
{
  ShipController::ShipController():command_struct()
  {

  }

  bool ShipController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh)
  {
    controller_nh.getParam("right_motor_command_topic",right_motor_command_topic);
    controller_nh.getParam("left_motor_command_topic",left_motor_command_topic);
    controller_nh.getParam("twist_topic",twist_topic);
    controller_nh.getParam("linear/max_velocity",max_linear_velocity);
    controller_nh.getParam("linear/min_velocity",min_linear_velocity);
    controller_nh.getParam("angular/max_velocity",max_angular_velocity);
    controller_nh.getParam("angular/min_velocity",min_angular_velocity);
    double p_gain,i_gain,d_gain;
    controller_nh.param<double>("linear/pid/p", p_gain, 0);
    controller_nh.param<double>("linear/pid/i", i_gain, 0);
    controller_nh.param<double>("linear/pid/d", d_gain, 0);
    setGains(p_gain,i_gain,d_gain,-1,1,pid_controller_linear);
    controller_nh.param<double>("angular/pid/p", p_gain, 0);
    controller_nh.param<double>("angular/pid/i", i_gain, 0);
    controller_nh.param<double>("angular/pid/d", d_gain, 0);
    setGains(p_gain,i_gain,d_gain,-1,1,pid_controller_angular);
    /*
    controller_nh.getParam("izz",izz);
    controller_nh.getParam("mass",mass);
    controller_nh.getParam("motor_distance",motor_distance);
    */
    controller_nh.getParam("right_motor_joint",right_motor_joint_name);
    controller_nh.getParam("left_motor_joint",left_motor_joint_name);
    controller_nh.getParam("/propeller/"+right_motor_joint_name+"/rotational_speed_effort",right_rotational_speed_effort);
    controller_nh.getParam("/propeller/"+left_motor_joint_name+"/rotational_speed_effort",left_rotational_speed_effort);
    left_motor_joint_cmd_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(controller_nh, left_motor_command_topic, 1));
    right_motor_joint_cmd_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(controller_nh, right_motor_command_topic, 1));
    sub_twist = controller_nh.subscribe(twist_topic, 1, &ShipController::twistCallback, this);
    sub_command = controller_nh.subscribe("cmd_vel", 1, &ShipController::cmdVelCallback, this);
  }

  ShipController::~ShipController()
  {
    sub_command.shutdown();
  }

  void ShipController::starting(const ros::Time& time)
  {
    command_struct.lin = 0;
    command_struct.ang = 0;
    command.initRT(command_struct);
    pid_controller_linear.reset();
    pid_controller_angular.reset();
  }

  void ShipController::update(const ros::Time& time, const ros::Duration& period)
  {
    command_struct = *(command.readFromRT());
    twist_struct = *(twist.readFromRT());
    if(left_motor_joint_cmd_publisher && left_motor_joint_cmd_publisher->trylock())
    {

    }
    if(right_motor_joint_cmd_publisher && right_motor_joint_cmd_publisher->trylock())
    {

    }
  }

  void ShipController::stopping(const ros::Time& time)
  {

  }

  void ShipController::setGains(double p, double i, double d, double i_max, double i_min,control_toolbox::Pid& target)
  {
    target.setGains(p,i,d,i_max,i_min);
  }

  void ShipController::cmdVelCallback(const geometry_msgs::Twist& msg)
  {
    if (isRunning())
    {
      if(msg.angular.z > max_angular_velocity)
      {
        command_struct.ang = max_angular_velocity;
      }
      else if(msg.angular.z < min_angular_velocity)
      {
        command_struct.ang = min_angular_velocity;
      }
      else
      {
        command_struct.ang = msg.angular.z;
      }

      if(msg.angular.z > max_linear_velocity)
      {
        command_struct.lin = max_linear_velocity;
      }
      else if(msg.angular.z < min_linear_velocity)
      {
        command_struct.lin = min_linear_velocity;
      }
      else
      {
        command_struct.lin = msg.linear.x;
      }
      command_struct.stamp = ros::Time::now();
      command.writeFromNonRT(command_struct);
      ROS_INFO_STREAM("Added values to command. " << "Ang: "   << command_struct.ang << ", " << "Lin: "   << command_struct.lin << ", " << "Stamp: " << command_struct.stamp);
    }
    else
    {
      ROS_ERROR("Can't accept new commands. Controller is not running.");
    }
  }

  void ShipController::twistCallback(const geometry_msgs::Twist& msg)
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
}

PLUGINLIB_EXPORT_CLASS(ship_controller::ShipController, controller_interface::ControllerBase);
