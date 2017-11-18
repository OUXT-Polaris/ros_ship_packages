//headers in this package
#include <pcl_object_recognition.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_ship_map_server_node");
  pcl_object_recognition object_recognition;
  ros::spin();
  return 0;
}
