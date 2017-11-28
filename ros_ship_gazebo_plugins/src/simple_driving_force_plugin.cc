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
#include <geometry_msgs/Twist.h>

namespace gazebo
{
  class simple_driving_force_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->LoadParams(sdf,"target_link",this->target_link);
      this->LoadParams(sdf,"target_joint",this->target_joint);
      std::string default_joint_states_topic = "/joint_states";
      this->LoadParams(sdf,"joint_state_topic",this->joint_state_topic,default_joint_states_topic);
      this->LoadParams(sdf,"driving_force_controller",this->driving_force_controller);
      nh.getParam(this->driving_force_controller+"/propeller/k0", k0);
      nh.getParam(this->driving_force_controller+"/propeller/k1", k1);
      nh.getParam(this->driving_force_controller+"/propeller/k2", k2);
      nh.getParam(this->driving_force_controller+"/propeller/fluid_density",fluid_density);
      nh.getParam(this->driving_force_controller+"/propeller/turning_radius", turning_radius);
      //nh.getParam(this->driving_force_controller+"/propeller/twist_topic",twist_topic);
      this->joint = this->model->GetJoint(this->target_joint);
      this->link = this->model->GetLink(target_link);
      driving_force_pub = nh.advertise<std_msgs::Float32>("/"+this->target_link+"/driving_force", 1);
      //twist_sub = nh.subscribe(twist_topic, 1, &simple_driving_force_plugin::twist_callback,this);
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&simple_driving_force_plugin::OnUpdate, this, _1));
    }

    double get_thrust(double rotational_speed,double inflow_rate)
    {
      if(rotational_speed == 0)
      {
        return 0;
      }
      if(rotational_speed > 0)
      {
        double Js = inflow_rate/rotational_speed*turning_radius;
        double Kt = k2*Js*Js + k1*Js + k0;
        double thrust = fluid_density*std::pow(rotational_speed,2)*std::pow(turning_radius,4)*Kt;
        return thrust;
      }
      if(rotational_speed < 0)
      {
        double Js = inflow_rate/rotational_speed*turning_radius;
        double Kt = k2*Js*Js + k1*Js + k0;
        double thrust = -fluid_density*std::pow(rotational_speed,2)*std::pow(turning_radius,4)*Kt;
        return thrust;
      }
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      inflow_rate = model->GetWorldLinearVel().x;
      std_msgs::Float32 driving_force_msg;
      double velocity = this->joint->GetVelocity(0);
      double driving_force = get_thrust(velocity,this->inflow_rate);
      this->link->AddForce(math::Vector3(driving_force, 0, 0));
      driving_force_msg.data = driving_force;
      driving_force_pub.publish(driving_force_msg);
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
    private: double rotational_speed_effort;
    private: std::string target_link,target_joint,joint_state_topic;
    private: physics::JointPtr joint;
    private: physics::ModelPtr model;
    private: physics::LinkPtr link;

    //ros publishers
    private: ros::NodeHandle nh;
    private: ros::Publisher driving_force_pub;
    //private: ros::Subscriber twist_sub;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    //propeller params
    std::string driving_force_controller;//,twist_topic;
    double k0,k1,k2;
    double fluid_density,turning_radius,inflow_rate;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(simple_driving_force_plugin)
}
