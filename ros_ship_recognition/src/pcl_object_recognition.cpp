#include <pcl_object_recognition.h>

pcl_object_recognition::pcl_object_recognition()
{
  nh_.getParam(ros::this_node::getName()+"/object_stl_file_path", stl_file_path_);
  pointcloud_sub_ = nh_.subscribe(ros::this_node::getName()+"/input_cloud", 1, &pcl_object_recognition::pointcloud_callback, this);
}

pcl_object_recognition::~pcl_object_recognition()
{

}

void pcl_object_recognition::pointcloud_callback(sensor_msgs::PointCloud2 pcl_input_cloud)
{

}
