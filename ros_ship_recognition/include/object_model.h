#ifndef OBJECT_MODEL_H_INCLUDED
#define OBJECT_MODEL_H_INCLUDED

//headers in pcl
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>

class object_model
{
public:
  object_model(pcl::PointCloud<pcl::PointXYZ>::Ptr model);
  ~object_model();
  pcl::PointCloud<pcl::SHOT352>::Ptr get_model_descriptors();
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_;
  pcl::PointCloud<pcl::Normal>::Ptr model_normals_;
  pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors_;
};
#endif //OBJECT_MODEL_H_INCLUDED
