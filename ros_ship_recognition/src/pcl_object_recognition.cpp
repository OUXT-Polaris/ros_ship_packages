#include <pcl_object_recognition.h>

//headers in pcl
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/correspondence.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

//headers in stl
#include<string>
#include<fstream>
#include<iostream>

pcl_object_recognition::pcl_object_recognition()
{
  nh_.getParam(ros::this_node::getName()+"/object_stl_file_path", stl_file_path_);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  object_pointcloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if(check_file_existence(stl_file_path_) == true)
  {
    pcl::io::loadPolygonFileSTL(stl_file_path_, *mesh);
    pcl::fromPCLPointCloud2(mesh->cloud, *object_pointcloud_);
    ROS_INFO_STREAM("load stl file from:" << stl_file_path_);
  }
  else
  {
    ROS_ERROR_STREAM("file is not exist:" << stl_file_path_);
    return;
  }
  object_model_ = new object_model(object_pointcloud_);
  pointcloud_sub_ = nh_.subscribe(ros::this_node::getName()+"/input_cloud", 1, &pcl_object_recognition::pointcloud_callback, this);
}

pcl_object_recognition::~pcl_object_recognition()
{

}

void pcl_object_recognition::pointcloud_callback(sensor_msgs::PointCloud2 input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(input_cloud,*scene_pointcloud);
  scene_model_ = new object_model(scene_pointcloud);
  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  match_search.setInputCloud(object_model_->get_model_descriptors());
  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for(size_t i = 0; i < scene_model_->get_model_descriptors()->size(); ++i)
  {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if(!pcl_isfinite(scene_model_->get_model_descriptors()->at(i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch(scene_model_->get_model_descriptors()->at(i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
    }
  }
  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;
}

bool pcl_object_recognition::check_file_existence(std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}
