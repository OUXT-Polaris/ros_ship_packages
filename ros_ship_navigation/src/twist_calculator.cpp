//headers in this package
#include <twist_calculator.h>

//headers for ros
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

twist_calculator::twist_calculator()
{
  tf_buffer = new tf2_ros::Buffer();
  tf_listener = new tf2_ros::TransformListener(*tf_buffer);
  nh.param<std::string>(ros::this_node::getName()+"/fix_velocity_frame", fix_velocity_frame, "world");
  nh.param<std::string>(ros::this_node::getName()+"/twist_frame", twist_frame, "base_link");
  nh.param<std::string>(ros::this_node::getName()+"/imu_frame", imu_frame, "imu");
  twist_pub = nh.advertise<geometry_msgs::Twist>("/twist", 1);
  fix_velocity_sub = nh.subscribe("/fix_velocity", 1, &twist_calculator::fix_velocity_callback, this);
  imu_sub = nh.subscribe("/imu", 1, &twist_calculator::imu_callback, this);
}

void twist_calculator::imu_callback(const sensor_msgs::Imu msg)
{
  geometry_msgs::TransformStamped transform_angular;
  geometry_msgs::Vector3Stamped angular_velocity_msg;
  angular_velocity_msg.header = msg.header;
  angular_velocity_msg.vector = msg.angular_velocity;
  try
  {
    transform_angular = tf_buffer->lookupTransform(imu_frame, fix_velocity_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::doTransform(angular_velocity_msg,angular_velocity,transform_angular);
}

void twist_calculator::fix_velocity_callback(const geometry_msgs::Vector3Stamped msg)
{
  geometry_msgs::TransformStamped transform_linear;
  try
  {
    transform_linear = tf_buffer->lookupTransform(twist_frame, fix_velocity_frame, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::doTransform(msg,linear_velocity,transform_linear);
  geometry_msgs::Twist twist_msg;
  twist_msg.linear = linear_velocity.vector;
  twist_msg.angular = angular_velocity.vector;
  twist_pub.publish(twist_msg);
}
