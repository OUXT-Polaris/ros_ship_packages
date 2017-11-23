#include <pcl_object_recognition.h>

//headers in pcl
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/correspondence.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>

//headers in stl
#include<string>
#include<fstream>
#include<iostream>

pcl_object_recognition::pcl_object_recognition()
{
  nh_.getParam(ros::this_node::getName()+"/object_stl_file_path", stl_file_path_);
  nh_.param<bool>(ros::this_node::getName()+"/use_hough_", use_hough_, true);
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
  if(use_hough_ == true)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame> ());

    pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(1);

    rf_est.setInputCloud(object_model_->get_model_keypoints());
    rf_est.setInputNormals(object_model_->get_model_normals());
    rf_est.setSearchSurface(object_model_->get_model());
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene_model_->get_model_keypoints());
    rf_est.setInputNormals(scene_model_->get_model_normals());
    rf_est.setSearchSurface(scene_model_->get_model());
    rf_est.compute(*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize(1);
    clusterer.setHoughThreshold(5.0);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(object_model_->get_model_keypoints());
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud(scene_model_->get_model_keypoints());
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(model_scene_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else
  {

  }
  //
  // Output results
  //
  ROS_INFO_STREAM("Model instances found: " << rototranslations.size ());
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);
  }
}

bool pcl_object_recognition::check_file_existence(std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}
