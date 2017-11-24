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
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_pointcloud_;
  std::string pointcloud_topic_;
  void pointcloud_callback(sensor_msgs::PointCloud2 input_cloud);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > recognize(object_model* target_object_model);
  void publish_messages(std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > results, std_msgs::Header header);
  inline bool check_file_existence(std::string& str);
  void read_parameters();
  object_model* load_object_model(std::string object_name, std::string stl_file_path, std::string marker_mesh_path);
  std::vector<object_model*> object_models_;
  object_model* scene_model_;
  geometry_msgs::Quaternion rot_to_quat(Eigen::Matrix3f rotation);
  inline float sign(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
};

#endif
