#ifndef DRIVING_FORCE_CONTROLLER_H_INCLUDED
#define DRIVING_FORCE_CONTROLLER_H_INCLUDED

//headers for controller_interface
#include <controller_interface/controller.h>

//headers for fardware_interface
#include <hardware_interface/joint_command_interface.h>

//headers for realtime_tools
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

//headers for ROS
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

//headers for boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread.hpp>

namespace driving_force_controller
{
  class DrivingForceController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    DrivingForceController();
    ~DrivingForceController();
  private:
    void twistCallback(const geometry_msgs::Twist& msg);
    void drivingForceCallback(const std_msgs::Float64& msg);
    double get_thrust(double rotational_speed,double inflow_rate);
    //void plot_characteristic_curve(double min_rotational_speed,double max_rotational_speed,double resolution_rotational_speed,
    //  double min_inflow_rate,double max_inflow_rate,double resolution_inflow_rate);
    double get_rotational_speed(double thrust,double inflow_rate);
    bool init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);

    boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64> > motor_command_publisher;

    struct Twist
    {
      double lin_x;
      double lin_y;
      double lin_z;
      double ang_x;
      double ang_y;
      double ang_z;
      ros::Time stamp;
      Twist() : lin_x(0.0), lin_y(0.0), lin_z(0.0), ang_x(0.0), ang_y(0.0), ang_z(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Twist> twist;
    Twist twist_struct;
    ros::Subscriber twist_sub;

    struct DrivingForce
    {
      double value;
      ros::Time stamp;
      DrivingForce() : value(0.0) {}
    };
    realtime_tools::RealtimeBuffer<DrivingForce> driving_force;
    DrivingForce driving_force_struct;
    ros::Subscriber driving_force_sub;

    //parameters
    std::string twist_topic,motor_command_topic,driving_force_command_topic,characteristic_curve_file_name;
    double turning_radius,k2,k1,k0,fluid_density;
    ros::ServiceClient plot_client;
    double tolerance,alpha;
    double max_rotational_speed,min_rotational_speed;
  };
}
#endif
