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
#include <std_msgs/Float32.h>

namespace gazebo
{
  class simple_driving_force_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->LoadParams(sdf,"rotational_speed_effort",this->rotational_speed_effort,1.0);
      this->LoadParams(sdf,"target_link",this->target_link);
      this->LoadParams(sdf,"target_joint",this->target_joint);
      std::string default_joint_states_topic = "/joint_states";
      this->LoadParams(sdf,"joint_state_topic",this->joint_state_topic,default_joint_states_topic);
      this->joint = this->model->GetJoint(this->target_joint);
      this->link = this->model->GetLink(target_link);
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      double velocity = this->joint->GetVelocity(0);
      this->link->AddForce(math::Vector3(1, 0, 0));
      ROS_ERROR_STREAM(velocity);
    }

    template <typename T>
    bool LoadParams(sdf::ElementPtr sdf,std::string key,T& param)
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
        param = boost::lexical_cast<T>(param_str);
      }
      catch(boost::bad_lexical_cast &)
      {
        ROS_WARN_STREAM("failed to casting " << key);
      }
      return true;
    }

    template <typename T>
    bool LoadParams(sdf::ElementPtr sdf,std::string key,T& param, T default_param)
    {
      std::string param_str;
      if(!sdf->HasElement(key))
      {
        param = default_param;
        ROS_INFO_STREAM("unavle to get " << key << " ,set default_value = " << default_param);
        return false;
      }
      else
      {
        param_str = sdf->GetElement(key)->GetValue()->GetAsString();
      }
      try
      {
        param = boost::lexical_cast<T>(param_str);
      }
      catch(boost::bad_lexical_cast &)
      {
        ROS_WARN_STREAM("failed to casting " << key);
        param = default_param;
      }
      return true;
    }
    private: double rotational_speed_effort;
    private: std::string target_link,target_joint,joint_state_topic;
    private: physics::JointPtr joint;
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;
    private: ros::NodeHandle nh;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(simple_driving_force_plugin)
}
