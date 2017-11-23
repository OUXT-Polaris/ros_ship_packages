#ifndef PCL_OBJECT_RECOGNITION_H_INCLUDED
#define PCL_OBJECT_RECOGNITION_H_INCLUDED

//headers in this package
#include <object_model.h>
#include <ros_ship_msgs/Objects.h>

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

//headers in pcl
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>

class pcl_object_recognition
{
public:
  pcl_object_recognition();
  ~pcl_object_recognition();
private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher detected_object_pub_,object_marker_pub_;
  std::string stl_file_path_,marker_mesh_path_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_;
  void pointcloud_callback(sensor_msgs::PointCloud2 input_cloud);
  inline bool check_file_existence(std::string& str);
  object_model* object_model_;
  object_model* scene_model_;
  bool use_hough_;
  std::string object_type_;
  geometry_msgs::Quaternion rot_to_quat(Eigen::Matrix3f rotation);
  inline float sign(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
};

#endif
