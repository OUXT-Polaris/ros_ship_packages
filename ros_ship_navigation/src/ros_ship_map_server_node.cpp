//headers in this package
#include <ros_ship_map_server.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_ship_map_server_node");
  ros_ship_map_server map_server;
  ros::spin();
  return 0;
}
