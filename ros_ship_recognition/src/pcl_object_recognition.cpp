#include <pcl_object_recognition.h>

//headers in ROS
#include <ros/package.h>

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
#include <pcl/recognition/cg/geometric_consistency.h>

//headers in stl
#include <string>
#include <fstream>
#include <iostream>

pcl_object_recognition::pcl_object_recognition()
{
  read_parameters();
  detected_object_pub_ = nh_.advertise<ros_ship_msgs::Objects>(ros::this_node::getName()+"/detected_objects", 1);
  object_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/detected_objects/marker", 1);
  pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1, &pcl_object_recognition::pointcloud_callback, this);
}

pcl_object_recognition::~pcl_object_recognition()
{

}

void pcl_object_recognition::pointcloud_callback(sensor_msgs::PointCloud2 input_cloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(input_cloud,*scene_pointcloud);
  scene_model_ = new object_model(scene_pointcloud,"scene");
  std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > results;
  for(auto object_model_itr = object_models_.begin(); object_model_itr != object_models_.end(); ++object_model_itr)
  {
    results.push_back(recognize(*object_model_itr));
  }
  publish_messages(results,input_cloud.header);
}

bool pcl_object_recognition::check_file_existence(std::string& str)
{
    std::ifstream ifs(str);
    return ifs.is_open();
}

void pcl_object_recognition::publish_messages(std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > results, std_msgs::Header header)
{
  ros_ship_msgs::Objects objects_msg;
  visualization_msgs::MarkerArray marker_array_msg;
  for(int i = 0; i < results.size(); i++)
  {
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations = results[i];
    for (size_t m = 0; m < rototranslations.size(); ++m)
    {
      visualization_msgs::Marker marker_msg;
      marker_msg.type = marker_msg.MESH_RESOURCE;
      ros_ship_msgs::Object object_msg;
      object_msg.type = object_models_[i]->get_name();
      // Print the rotation matrix and translation vector
      Eigen::Matrix3f rotation = rototranslations[m].block<3,3>(0, 0);
      Eigen::Vector3f translation = rototranslations[m].block<3,1>(0, 3);
      object_msg.pose.header = header;
      object_msg.pose.pose.position.x = translation(0);
      object_msg.pose.pose.position.y = translation(1);
      object_msg.pose.pose.position.z = translation(2);
      object_msg.pose.pose.orientation = rot_to_quat(rotation);
      objects_msg.objects.push_back(object_msg);
      marker_msg.header = header;
      marker_msg.pose = object_msg.pose.pose;
      marker_msg.scale.x = 1;
      marker_msg.scale.y = 1;
      marker_msg.scale.z = 1;
      marker_msg.frame_locked = true;
      marker_msg.mesh_resource = "file://"+object_models_[i]->get_marker_mesh_path();
      marker_msg.color.r = 1;
      marker_msg.color.g = 1;
      marker_msg.color.b = 1;
      marker_msg.color.a = 0.5;
      marker_array_msg.markers.push_back(marker_msg);
    }
  }
  detected_object_pub_.publish(objects_msg);
  object_marker_pub_.publish(marker_array_msg);
}

std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > pcl_object_recognition::recognize(object_model* target_object_model)
{
  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
  pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  match_search.setInputCloud(target_object_model->get_model_descriptors());
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
  //
  //  Compute (Keypoints) Reference Frames only for Hough
  //
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame> ());
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame> ());

  pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(1);

  rf_est.setInputCloud(target_object_model->get_model_keypoints());
  rf_est.setInputNormals(target_object_model->get_model_normals());
  rf_est.setSearchSurface(target_object_model->get_model());
  rf_est.compute(*model_rf);

  rf_est.setInputCloud(scene_model_->get_model_keypoints());
  rf_est.setInputNormals(scene_model_->get_model_normals());
  rf_est.setSearchSurface(scene_model_->get_model());
  rf_est.compute(*scene_rf);

  //  Clustering
  pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
  clusterer.setHoughBinSize(1.0);
  clusterer.setHoughThreshold(5.0);
  clusterer.setUseInterpolation(true);
  clusterer.setUseDistanceWeight(false);

  clusterer.setInputCloud(target_object_model->get_model_keypoints());
  clusterer.setInputRf(model_rf);
  clusterer.setSceneCloud(scene_model_->get_model_keypoints());
  clusterer.setSceneRf(scene_rf);
  clusterer.setModelSceneCorrespondences(model_scene_corrs);
  clusterer.recognize (rototranslations, clustered_corrs);
  return rototranslations;
}

geometry_msgs::Quaternion pcl_object_recognition::rot_to_quat(Eigen::Matrix3f rotation)
{
  geometry_msgs::Quaternion quat;
  float q0 = ( rotation(0,0)+rotation(1,1)+rotation(2,2)+1.0f)/4.0f;
  float q1 = ( rotation(0,0)-rotation(1,1)-rotation(2,2)+1.0f)/4.0f;
  float q2 = (-rotation(0,0)+rotation(1,1)-rotation(2,2)+1.0f)/4.0f;
  float q3 = (-rotation(0,0)-rotation(1,1)+rotation(2,2)+1.0f)/4.0f;
  if(q0 < 0.0f) q0 = 0.0f;
  if(q1 < 0.0f) q1 = 0.0f;
  if(q2 < 0.0f) q2 = 0.0f;
  if(q3 < 0.0f) q3 = 0.0f;
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if(q0 >= q1 && q0 >= q2 && q0 >= q3)
  {
    q0 *= +1.0f;
    q1 *= sign(rotation(2,1)-rotation(1,2));
    q2 *= sign(rotation(0,2)-rotation(2,0));
    q3 *= sign(rotation(1,0)-rotation(0,1));
  }
  else if(q1 >= q0 && q1 >= q2 && q1 >= q3)
  {
      q0 *= sign(rotation(2,1)-rotation(1,2));
      q1 *= +1.0f;
      q2 *= sign(rotation(1,0)+rotation(0,1));
      q3 *= sign(rotation(0,2)+rotation(2,0));
  }
  else if(q2 >= q0 && q2 >= q1 && q2 >= q3)
  {
      q0 *= sign(rotation(0,2)-rotation(2,0));
      q1 *= sign(rotation(1,0)+rotation(0,1));
      q2 *= +1.0f;
      q3 *= sign(rotation(2,1)+rotation(1,2));
  }
  else if(q3 >= q0 && q3 >= q1 && q3 >= q2)
  {
      q0 *= sign(rotation(1,0)-rotation(0,1));
      q1 *= sign(rotation(0,2)+rotation(2.0));
      q2 *= sign(rotation(2,1)+rotation(1,2));
      q3 *= +1.0f;
  }
  else
  {
      ROS_ERROR_STREAM("Failed to convert quaternion");
  }
  return quat;
}

object_model* pcl_object_recognition::load_object_model(std::string object_name, std::string stl_file_path, std::string marker_mesh_path)
{
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  if(check_file_existence(stl_file_path) == true)
  {
    pcl::io::loadPolygonFileSTL(stl_file_path, *mesh);
    pcl::fromPCLPointCloud2(mesh->cloud, *object_pointcloud);
    ROS_INFO_STREAM("load stl file from:" << stl_file_path);
  }
  else
  {
    ROS_ERROR_STREAM("file is not exist:" << stl_file_path);
    //return;
  }
  object_model* obj_model = new object_model(object_pointcloud,object_name,stl_file_path,marker_mesh_path);
  return obj_model;
}

void pcl_object_recognition::read_parameters()
{
  XmlRpc::XmlRpcValue parameters;
  nh_.getParam(ros::this_node::getName(), parameters);
  pointcloud_topic_ = std::string(parameters["input_cloud"]);
  XmlRpc::XmlRpcValue target_objects_params = parameters["target_objects"];
  std::string resource_path = ros::package::getPath("ros_ship_recognition") + "/data/";
  for(auto object_param_itr = target_objects_params.begin(); object_param_itr != target_objects_params.end(); ++object_param_itr)
  {
    std::string stl_path = resource_path+std::string(object_param_itr->second["stl_filename"]);
    std::string mesh_path = resource_path+std::string(object_param_itr->second["marker_filename"]);
    object_models_.push_back(load_object_model(object_param_itr->first,stl_path,mesh_path));
  }
}
