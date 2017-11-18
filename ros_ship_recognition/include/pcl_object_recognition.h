#ifndef PCL_OBJECT_RECOGNITION_H_INCLUDED
#define PCL_OBJECT_RECOGNITION_H_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class pcl_object_recognition
{
public:
  pcl_object_recognition();
  ~pcl_object_recognition();
private:
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_sub_;
  std::string stl_file_path_;
  void pointcloud_callback(sensor_msgs::PointCloud2 pcl_input_cloud);
};

#endif
