#ifndef TWIST_CALCULATOR_H_INCLUDED
#define TWIST_CALCULATOR_H_INCLUDED

//headers for ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class twist_calculator
{
public:
  twist_calculator();
private:
  //callback functions
  void fix_velocity_callback(const geometry_msgs::Vector3Stamped msg);
  void imu_callback(const sensor_msgs::Imu msg);
  //publishers and Subscribers
  ros::Subscriber fix_velocity_sub,imu_sub;
  ros::Publisher twist_pub;
  ros::NodeHandle nh;
  //tf
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  //parameters
  std::string fix_velocity_frame,twist_frame,imu_frame;
  //datas
  geometry_msgs::Vector3Stamped angular_velocity;
  geometry_msgs::Vector3Stamped linear_velocity;
};

#endif
