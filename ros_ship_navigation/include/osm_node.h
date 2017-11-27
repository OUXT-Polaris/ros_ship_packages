#ifndef OSM_NODE_H_INCLUDED
#define OSM_NODE_H_INCLUDED

class osm_node
{
public:
  osm_node(double latitude,double longitude,double id):latitude_(latitude),longitude_(longitude),id_(id){}
  ~osm_node();
  const double latitude_;
  const double longitude_;
  const long id_;
};

#endif //NODE_H_INCLUDED
