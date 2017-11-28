#ifndef GEOGRAPHIC_MAP_SERVER_H_INCLUDED
#define GEOGRAPHIC_MAP_SERVER_H_INCLUDED

//headers in this package
#include <osm_node.h>
#include <osm_way.h>

//headers in ROS
#include <ros/ros.h>

//headers in stl
#include <vector>

class geographic_map_server
{
public:
  geographic_map_server();
  ~geographic_map_server();
private:
  void parse_osm();
  void get_osm_way();
  ros::NodeHandle nh_;
  std::string osm_filepath_;
  //datas for osm
  std::vector<osm_node*> osm_nodes_;
  std::vector<osm_way*> osm_ways_;
};
#endif //GEOGRAPHIC_MAP_SERVER_H_INCLUDED
