#ifndef OBJECT_MODEL_H_INCLUDED
#define OBJECT_MODEL_H_INCLUDED

//headers in pcl
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>

class object_model
{
public:
  object_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model, std::string object_name);
  object_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model, std::string object_name, std::string stl_file_path, std::string marker_mesh_path);
  ~object_model();
  std::string get_name();
  std::string get_stl_file_path();
  std::string get_marker_mesh_path();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_model();
  pcl::PointCloud<pcl::Normal>::Ptr get_model_normals();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_model_keypoints();
  pcl::PointCloud<pcl::SHOT352>::Ptr get_model_descriptors();
private:
  std::string object_name_,stl_file_path_,marker_mesh_path_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_;
  pcl::PointCloud<pcl::Normal>::Ptr model_normals_;
  pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors_;
};
#endif //OBJECT_MODEL_H_INCLUDED
