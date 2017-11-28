//headers in this package
#include <driving_force_controller.h>
#include <ros_ship_visualization/PlotCharacteristicCurve.h>

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
#include <boost/thread.hpp>
#include <boost/algorithm/clamp.hpp>

//headers for ROS
#include <ros/package.h>

//headers for STL
#include <vector>

namespace driving_force_controller
{
  DrivingForceController::DrivingForceController()
  {

  }

  DrivingForceController::~DrivingForceController()
  {

  }

  void DrivingForceController::drivingForceCallback(const std_msgs::Float64& msg)
  {
    if (isRunning())
    {
      driving_force_struct.value = msg.data;
      driving_force.writeFromNonRT(driving_force_struct);
    }
    else
    {
      ROS_WARN_STREAM("Can't accept new commands. Controller is not running.");
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
      ROS_WARN_STREAM("Can't accept new commands. Controller is not running.");
    }
  }

  double DrivingForceController::get_thrust(double rotational_speed,double inflow_rate)
  {
    if(rotational_speed == 0)
    {
      return 0;
    }
    if(rotational_speed > 0)
    {
      double Js = inflow_rate/rotational_speed*turning_radius;
      double Kt = k2*Js*Js + k1*Js + k0;
      double thrust = fluid_density*std::pow(rotational_speed,2)*std::pow(turning_radius,4)*Kt;
      return thrust;
    }
    if(rotational_speed < 0)
    {
      double Js = inflow_rate/rotational_speed*turning_radius;
      double Kt = k2*Js*Js + k1*Js + k0;
      double thrust = -fluid_density*std::pow(rotational_speed,2)*std::pow(turning_radius,4)*Kt;
      return thrust;
    }
  }

  bool DrivingForceController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh)
  {
    controller_nh.getParam("twist_topic",twist_topic);
    controller_nh.getParam("motor_command_topic",motor_command_topic);
    controller_nh.getParam("driving_force_command_topic",driving_force_command_topic);
    controller_nh.getParam("propeller/turning_radius",turning_radius);
    controller_nh.getParam("propeller/k2",k2);
    controller_nh.getParam("propeller/k1",k1);
    controller_nh.getParam("propeller/k0",k0);
    controller_nh.getParam("propeller/fluid_density",  fluid_density);
    controller_nh.getParam("characteristic_curve_file_name",characteristic_curve_file_name);
    controller_nh.getParam("gradient_descent/tolerance",tolerance);
    controller_nh.getParam("gradient_descent/alpha",alpha);
    controller_nh.getParam("max_rotational_speed",max_rotational_speed);
    controller_nh.getParam("min_rotational_speed",min_rotational_speed);
    //plot_characteristic_curve(0,30,1,0,30,5);
    //boost::thread plot_thread(boost::bind(&DrivingForceController::plot_characteristic_curve,this,0,30,1,0,30,5));
    plot_client = controller_nh.serviceClient<ros_ship_visualization::PlotCharacteristicCurve>("/plot_characteristic_curve");
    ros_ship_visualization::PlotCharacteristicCurve plot_service_message;
    plot_service_message.request.fluid_density = fluid_density;
    plot_service_message.request.turning_radius = turning_radius;
    plot_service_message.request.k0 = k0;
    plot_service_message.request.k1 = k1;
    plot_service_message.request.k2 = k2;
    plot_service_message.request.file_name = characteristic_curve_file_name;
    plot_service_message.request.min_inflow_rate = 0;
    plot_service_message.request.max_inflow_rate = 3;
    plot_service_message.request.resolution_inflow_rate = 1;
    plot_service_message.request.min_rotational_speed = min_rotational_speed;
    plot_service_message.request.max_rotational_speed = max_rotational_speed;
    plot_service_message.request.resolution_rotational_speed = 0.01;
    plot_client.call(plot_service_message);
    motor_command_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64>(controller_nh, motor_command_topic, 1));
    twist_sub = controller_nh.subscribe(twist_topic, 1, &DrivingForceController::twistCallback, this);
    driving_force_sub = controller_nh.subscribe(driving_force_command_topic, 1, &DrivingForceController::drivingForceCallback, this);
  }

  double DrivingForceController::get_rotational_speed(double thrust,double inflow_rate)
  {
    double rotational_speed = 0;
    double error = 0;
    error = thrust-get_thrust(rotational_speed,inflow_rate);
    while(std::fabs(error) > tolerance)
    {
      rotational_speed = rotational_speed+error*alpha;
      error = thrust-get_thrust(rotational_speed,inflow_rate);
    }
    return rotational_speed;
  }

  void DrivingForceController::starting(const ros::Time& time)
  {
    twist_struct.ang_x = 0;
    twist_struct.ang_y = 0;
    twist_struct.ang_z = 0;
    twist_struct.lin_x = 0;
    twist_struct.lin_y = 0;
    twist_struct.lin_z = 0;
    twist.initRT(twist_struct);
    driving_force_struct.value = 0;
    driving_force.initRT(driving_force_struct);
  }

  void DrivingForceController::update(const ros::Time& time, const ros::Duration& period)
  {
    driving_force_struct = *(driving_force.readFromRT());
    twist_struct = *(twist.readFromRT());
    if(motor_command_publisher && motor_command_publisher->trylock())
    {
      double target_rotational_speed = get_rotational_speed(driving_force_struct.value,twist_struct.lin_x);
      target_rotational_speed = boost::algorithm::clamp(target_rotational_speed,min_rotational_speed,max_rotational_speed);
      motor_command_publisher->msg_.data = target_rotational_speed;
      motor_command_publisher->unlockAndPublish();
    }
  }
  void DrivingForceController::stopping(const ros::Time& time)
  {

  }
}

PLUGINLIB_EXPORT_CLASS(driving_force_controller::DrivingForceController, controller_interface::ControllerBase);
