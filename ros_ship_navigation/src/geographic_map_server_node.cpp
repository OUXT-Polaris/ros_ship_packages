//headers in this package
#include <geographic_map_server.h>

//headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "geographic_map_server_node");
  geographic_map_server map_server;
  ros::spin();
  return 0;
}
