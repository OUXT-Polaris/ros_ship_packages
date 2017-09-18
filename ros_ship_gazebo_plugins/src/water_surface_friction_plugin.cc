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

namespace gazebo
{
  class water_surface_friction_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->LoadParams(sdf,"target_link",this->target_link_name);
      this->LoadParams(sdf,"mu_linear",this->mu_linear);
      this->LoadParams(sdf,"mu_angular",this->mu_angular);
      this->LoadParams(sdf,"cruising_speed",this->cruising_speed,(float)1.0);
      this->LoadParams(sdf,"cruising_speed_rot",this->cruising_speed_rot,(float)0.3);
      this->target_link = this->model->GetLink(target_link_name);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&water_surface_friction_plugin::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      math::Vector3 angular_vel = this->target_link->GetRelativeAngularVel();
      math::Vector3 linear_vel = this->target_link->GetRelativeLinearVel();
      math::Vector3 linear_friction_force,angular_friction_torque;
      if(linear_vel.x > this->cruising_speed)
      {
        linear_friction_force.x = -1*std::pow(linear_vel.x,2)*this->mu_linear;
      }
      else
      {
        linear_friction_force.x = -1*linear_vel.x*this->mu_linear;
      }
      if(linear_vel.y > this->cruising_speed)
      {
        linear_friction_force.y = -1*std::pow(linear_vel.y,2)*this->mu_linear;
      }
      else
      {
        linear_friction_force.y = -1*linear_vel.y*this->mu_linear;
      }
      if(linear_vel.z > this->cruising_speed)
      {
        linear_friction_force.z = -1*std::pow(linear_vel.z,2)*this->mu_linear;
      }
      else
      {
        linear_friction_force.z = -1*linear_vel.z*this->mu_linear;
      }

      if(angular_vel.x > this->cruising_speed)
      {
        angular_friction_torque.x = -1*std::pow(angular_vel.x,2)*this->mu_angular;
      }
      else
      {
        angular_friction_torque.x = -1*angular_vel.x*this->mu_angular;
      }
      if(angular_vel.x > this->cruising_speed)
      {
        angular_friction_torque.x = -1*std::pow(angular_vel.x,2)*this->mu_angular;
      }
      else
      {
        angular_friction_torque.z = -1*angular_vel.x*this->mu_angular;
      }
      if(angular_vel.z > this->cruising_speed)
      {
        angular_friction_torque.z = -1*std::pow(angular_vel.z,2)*this->mu_angular;
      }
      else
      {
        angular_friction_torque.z = -1*angular_vel.z*this->mu_angular;
      }
      this->target_link->AddRelativeForce(linear_friction_force);
      this->target_link->AddRelativeTorque(angular_friction_torque);
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
        ROS_INFO_STREAM("unable to get " << key << " ,set default_value = " << default_param);
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

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the model
    private: physics::ModelPtr model;
    // Pointer to the link
    private: physics::LinkPtr target_link;
    //parameters
    private: std::string target_link_name;
    private: float mu_linear,mu_angular,cruising_speed,cruising_speed_rot;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(water_surface_friction_plugin)
}
