//headers in this package
#include <twist_calculator.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "twist_from_fix_velocity_node");
  twist_calculator calculator;
  ros::spin();
  return 0;
}
