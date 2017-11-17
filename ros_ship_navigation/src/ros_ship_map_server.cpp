//headers in this package
#include <ros_ship_map_server.h>

//headers in tf2
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

//headers in pcl
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros_ship_map_server::ros_ship_map_server()
{
  map_width_ = 200;
  map_height_ = 200;
  map_resolution_ = 0.1;
  tf_buffer_ = new tf2_ros::Buffer();
  tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
  create_map_meta_data();
  //nh_.param<int>(ros::this_node::getName()+"/history_length", history_length_, 5);
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(ros::this_node::getName()+"/map", 1);
  pointcloud_sub_ = nh_.subscribe(ros::this_node::getName()+"/input_cloud", 1, &ros_ship_map_server::pointcloud_callback, this);
}

ros_ship_map_server::~ros_ship_map_server()
{

}

void ros_ship_map_server::pointcloud_callback(sensor_msgs::PointCloud2 input_cloud)
{
  //transform input cloud to base_link frame
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_->lookupTransform("base_footprint", input_cloud.header.frame_id, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }
  tf2::doTransform(input_cloud, input_cloud, transform_stamped);

  //clustring point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input_cloud, *pcl_input_cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcl_input_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.3);
  ec.setMinClusterSize(30);
  ec.setMaxClusterSize(10000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_input_cloud);
  ec.extract(cluster_indices);
  int i = 0;
  for (auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    i++;
  }
  ROS_INFO_STREAM(i << " clusters are found!!");


  //input map data
  input_map_data();
  map_pub_.publish(map_data_);
  /*
  pcl::PointCloud<pcl::PointXYZ> pcl_input_cloud_filtered;
  //set pass through fileter for input cloud
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(*pcl_input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  */
}

void ros_ship_map_server::create_map_meta_data()
{
  nav_msgs::MapMetaData map_meta_data;
  map_meta_data.width = map_width_;
  map_meta_data.height = map_height_;
  map_meta_data.resolution = map_resolution_;
  map_meta_data.origin.position.x = -(double)map_height_*map_resolution_*0.5;
  map_meta_data.origin.position.y = -(double)map_width_*map_resolution_*0.5;
  map_meta_data.origin.position.z = 0;
  map_meta_data.origin.orientation.x = 0;
  map_meta_data.origin.orientation.y = 0;
  map_meta_data.origin.orientation.z = 0;
  map_meta_data.origin.orientation.w = 1;
  map_data_.info = map_meta_data;
  //map_data_.data.push_back(0);
}

void ros_ship_map_server::input_map_data()
{
  map_data_.header.frame_id = "base_footprint";
  map_data_.header.stamp = ros::Time::now();
  map_data_.data.clear();
  for(int i = 0;i < map_width_ ;i++)
  {
    for(int m = 0;m < map_height_ ;m++)
    {
      map_data_.data.push_back(0);
    }
  }
}
