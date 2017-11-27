#include <geographic_map_server.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

geographic_map_server::geographic_map_server()
{
  nh_.getParam(ros::this_node::getName()+"/osm_filepath", osm_filepath_);
  parse_osm();
}

geographic_map_server::~geographic_map_server()
{

}

void geographic_map_server::parse_osm()
{
  using namespace boost::property_tree;
  ptree pt;
  read_xml(osm_filepath_, pt);
  //parse osm file and get osm_node infomation
  for(auto itr : pt.get_child("osm"))
  {
    if(itr.first == "node")
    {
      double lat = itr.second.get<double>("<xmlattr>.lat");
      double lon = itr.second.get<double>("<xmlattr>.lon");
      long id = itr.second.get<long>("<xmlattr>.id");
      osm_node* osm_node_ptr = new osm_node(lat,lon,id);
      osm_nodes_.push_back(osm_node_ptr);
    }
    //parse osm file and get osm_way infomation
    if(itr.first == "way")
    {
      for(auto way_data_itr : itr.second )
      {
        //parse tag
        if(way_data_itr.first == "tag")
        {
          //get areas of natural = water
          if(way_data_itr.second.get<std::string>("<xmlattr>.k") == "natural" && way_data_itr.second.get<std::string>("<xmlattr>.v") == "water")
          {
            
          }
        }
        if(way_data_itr.first == "nd")
        {
          long id = way_data_itr.second.get<long>("<xmlattr>.ref");
        }
      }
    }
  }
}
