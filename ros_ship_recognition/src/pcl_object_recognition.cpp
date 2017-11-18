#include <pcl_object_recognition.h>

//headers in pcl
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

//headers in stl
#include<string>
#include<fstream>
#include<iostream>

pcl_object_recognition::pcl_object_recognition()
{
  nh_.getParam(ros::this_node::getName()+"/object_stl_file_path", stl_file_path_);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  if(check_file_existence(stl_file_path_) == true)
  {
    pcl::io::loadPolygonFileSTL(stl_file_path_, *mesh);
    pcl::fromPCLPointCloud2(mesh->cloud, object_pointcloud_);
    ROS_INFO_STREAM("load stl file from:" << stl_file_path_);
  }
  else
  {
    ROS_ERROR_STREAM("file is not exist:" << stl_file_path_);
    return;
  }
  pointcloud_sub_ = nh_.subscribe(ros::this_node::getName()+"/input_cloud", 1, &pcl_object_recognition::pointcloud_callback, this);
}

pcl_object_recognition::~pcl_object_recognition()
{

}

void pcl_object_recognition::pointcloud_callback(sensor_msgs::PointCloud2 pcl_input_cloud)
{

}

bool pcl_object_recognition::check_file_existence(std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}
