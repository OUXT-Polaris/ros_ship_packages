//headers in this package
#include <driving_force_controller.h>
#include <matplotlibcpp.h>

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

  double DrivingForceController::get_thrust(double rotational_speed,double inflow_rate)
  {
    double Js = inflow_rate/rotational_speed*turning_radius;
    double Kt = k2*Js*Js + k1*Js + k0;
    double thrust = fluid_density*std::pow(rotational_speed,2)*std::pow(turning_radius,4)*Kt;
    return thrust;
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
    draw_characteristic_curve(0,30,1,0,30,5);
    motor_command_publisher.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32>(controller_nh, motor_command_topic, 1));
    sub_twist = controller_nh.subscribe(twist_topic, 1, &DrivingForceController::twistCallback, this);
    driving_force_sub = controller_nh.subscribe(driving_force_command_topic, 1, &DrivingForceController::drivingForceCallback, this);
  }

  void DrivingForceController::draw_characteristic_curve(double min_rotational_speed,double max_rotational_speed,double resolution_rotational_speed,
    double min_inflow_rate,double max_inflow_rate,double resolution_inflow_rate)
  {
    std::vector<double> x_data,y_data;
    for(double inflow_rate = min_inflow_rate;inflow_rate<=max_inflow_rate;inflow_rate=inflow_rate+resolution_inflow_rate)
    {
      for(double rotational_speed = min_rotational_speed;rotational_speed<=max_rotational_speed;rotational_speed=rotational_speed+resolution_rotational_speed)
      {
        double thrust = get_thrust(rotational_speed,inflow_rate);
        x_data.push_back(rotational_speed);
        y_data.push_back(thrust);
      }
      matplotlibcpp::named_plot("ship speed:"+std::to_string(inflow_rate)+"[m/s]",x_data, y_data);
      x_data.clear();
      y_data.clear();
    }
    std::string package_path = ros::package::getPath("ros_ship_control");
    matplotlibcpp::xlabel("Rotational speed [Hz]");
    matplotlibcpp::ylabel("Thrust [N]");
    matplotlibcpp::legend_upper_left();
    matplotlibcpp::grid(true);
    matplotlibcpp::save(package_path+"/data/"+characteristic_curve_file_name+".eps");
    matplotlibcpp::save(package_path+"/data/"+characteristic_curve_file_name+".jpeg");
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
      motor_command_publisher->msg_.data = 0;
      motor_command_publisher->unlockAndPublish();
    }
  }
  void DrivingForceController::stopping(const ros::Time& time)
  {

  }
}

PLUGINLIB_EXPORT_CLASS(driving_force_controller::DrivingForceController, controller_interface::ControllerBase);
