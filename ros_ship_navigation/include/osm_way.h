#ifndef OSM_WAY_H_INCLUDED
#define OSM_WAY_H_INCLUDED

//headers in this package
#include "osm_node.h"

//headers in stl
#include <vector>

class osm_way
{
public:
  osm_way(std::vector<osm_node*> osm_nodes):osm_nodes_(osm_nodes){}
  const std::vector<osm_node*> osm_nodes_;
  const std::string natural;
};
#endif //OSM_WAY_H_INCLUDED
