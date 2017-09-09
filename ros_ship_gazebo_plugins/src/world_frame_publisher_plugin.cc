//headers for gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
//headers for sdf
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
//headers fot boost
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
//headers for ROS
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace gazebo
{
  class world_frame_publisher_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      if(this->LoadParams(sdf,"reference_link",reference_link))
      {
        this->link = this->model->GetLink(reference_link);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&world_frame_publisher_plugin::OnUpdate, this, _1));
      }
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      this->link_pose = this->link->GetWorldPose();
      geometry_msgs::TransformStamped transform_msg;
      transform_msg.header.stamp = ros::Time::now();
      transform_msg.header.frame_id = "gazebo_world";
      transform_msg.child_frame_id = reference_link;
      transform_msg.transform.translation.x = this->link_pose.pos.x;
      transform_msg.transform.translation.y = this->link_pose.pos.y;
      transform_msg.transform.translation.z = this->link_pose.pos.z;
      transform_msg.transform.rotation.x = this->link_pose.rot.x;
      transform_msg.transform.rotation.y = this->link_pose.rot.y;
      transform_msg.transform.rotation.z = this->link_pose.rot.z;
      transform_msg.transform.rotation.w = this->link_pose.rot.w;
      broadcaster.sendTransform(transform_msg);
    }

    public: bool LoadParams(sdf::ElementPtr sdf,std::string key,std::string& param)
    {
      if(!sdf->HasElement(key))
      {
          return false;
      }
      else
      {
          param = sdf->GetElement(key)->GetValue()->GetAsString();
      }
      return true;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    //Pointer to the link
    private: physics::LinkPtr link;
    private: math::Pose link_pose;

    //Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //tf broadcaster
    private: tf2_ros::TransformBroadcaster broadcaster;

    //parameters
    private: std::string reference_link;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(world_frame_publisher_plugin)
}
