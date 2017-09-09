//headers for stl
#include <sstream>
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
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
  class simple_buoyancy_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      if(this->LoadParams(sdf,"target_link",target_link))
      {
        this->link = this->model->GetLink(target_link);
        nh.param<float>("water_surface_height", this->water_surface_height,1.0);
        //this->LoadParams(sdf,"water_surface_height",this->water_surface_height,0);
        this->LoadParams(sdf,"cob_x",this->cob_x);
        this->LoadParams(sdf,"cob_y",this->cob_y);
        this->LoadParams(sdf,"cob_z",this->cob_z);
        this->LoadParams(sdf,"bbox_x",this->bbox_x);
        this->LoadParams(sdf,"bbox_y",this->bbox_y);
        this->LoadParams(sdf,"bbox_z",this->bbox_z);
        this->LoadParams(sdf,"publish_data",this->publish_data);

        if(!this->link)
        {
          ROS_ERROR_STREAM("Cannot find target link!! Check your URDF and gzclient panels");
        }
        else
        {
          if(this->publish_data = true)
          {
            this->bounding_box_pub = nh.advertise<visualization_msgs::Marker>("/"+target_link+"/bounding_box", 1);
            this->buoyancy_pub = nh.advertise<std_msgs::Float32>("/"+target_link+"/buoyancy", 1);
          }
          this->target_link_bounding_box = this->link->GetBoundingBox();
          this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&simple_buoyancy_plugin::OnUpdate, this, _1));
        }
      }
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      link_pose = this->link->GetWorldPose();
      std_msgs::Float32 buoyancy;
      if((link_pose.pos.z+this->cob_z-this->bbox_z/2) >= this->water_surface_height)
      {
        buoyancy.data = 0;
        this->link->AddForce(math::Vector3(0, 0, 0));
      }
      else
      {
        buoyancy.data = (this->water_surface_height-(link_pose.pos.z+this->cob_z-this->bbox_z/2))*9.8*1000*this->bbox_x*this->bbox_y;
        this->link->AddForce(math::Vector3(0, 0, buoyancy.data));
        //this->link->AddForceAtWorldPosition(math::Vector3(0, 0, buoyancy.data),link_pose.pos);
      }
      if(this->publish_data == true)
      {
        this->PublishMarker();
        this->buoyancy_pub.publish(buoyancy);
      }
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

    public: bool LoadParams(sdf::ElementPtr sdf,std::string key,float& param)
    {
      std::string param_str;
      if(!sdf->HasElement(key))
      {
        ROS_WARN_STREAM("failed to get " << key);
        return false;
      }
      else
      {
        param_str = sdf->GetElement(key)->GetValue()->GetAsString();
      }
      try
      {
        param = boost::lexical_cast<float>(param_str);
      }
      catch(boost::bad_lexical_cast &)
      {
        ROS_WARN_STREAM("failed to casting " << key);
        param = 0;
      }
      return true;
    }

    public: bool LoadParams(sdf::ElementPtr sdf,std::string key,float& param, float default_param)
    {
      std::string param_str;
      if(!sdf->HasElement(key))
      {
        ROS_WARN_STREAM("failed to get " << key);
        param = default_param;
        return false;
      }
      else
      {
        param_str = sdf->GetElement(key)->GetValue()->GetAsString();
      }
      try
      {
        param = boost::lexical_cast<float>(param_str);
      }
      catch(boost::bad_lexical_cast &)
      {
        ROS_WARN_STREAM("failed to casting " << key);
        param = default_param;
      }
      return true;
    }

    public: bool LoadParams(sdf::ElementPtr sdf,std::string key,bool& param)
    {
      std::string param_str;
      if(!sdf->HasElement(key))
      {
        ROS_WARN_STREAM("failed to get " << key);
        return false;
      }
      else
      {
        param_str = sdf->GetElement(key)->GetValue()->GetAsString();
      }
      std::istringstream(param_str) >> std::boolalpha >> param;
      return true;
    }

    private: void PublishMarker()
    {
      visualization_msgs::Marker marker;
      marker.type = marker.CUBE;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = target_link;
      marker.action = marker.ADD;
      marker.color.r = 0;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 0.5;
      marker.frame_locked = true;
      marker.scale.x = bbox_x;
      marker.scale.y = bbox_y;
      marker.scale.z = bbox_z;
      marker.pose.position.x = cob_x;
      marker.pose.position.y = cob_y;
      marker.pose.position.z = cob_z;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;
      bounding_box_pub.publish(marker);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    //Pointer to the link
    private: physics::LinkPtr link;
    private: math::Pose link_pose;

    //bounding box of target link
    private: math::Box target_link_bounding_box;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //parameters
    private: std::string target_link;
    private: float water_surface_height;
    private: float cob_x,cob_y,cob_z,bbox_x,bbox_y,bbox_z;
    private: bool publish_data;

    //publishers
    private: ros::Publisher bounding_box_pub,buoyancy_pub;
    private: ros::NodeHandle nh;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(simple_buoyancy_plugin)
}
