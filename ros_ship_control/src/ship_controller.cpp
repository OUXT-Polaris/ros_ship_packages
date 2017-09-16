/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar
 */

#include <ship_controller.h>

namespace ship_controller
{
  ShipController::ShipController()
  {

  }

  bool ShipController::init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& root_nh,ros::NodeHandle& controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 10.0);
    std::size_t id = complete_ns.find_last_of("/");
    name = complete_ns.substr(id + 1);
    ROS_INFO_STREAM_NAMED(name, "Controller state will be published at " << publish_rate << "Hz.");
  }

  void ShipController::starting(const ros::Time& time)
  {

  }

  void ShipController::update(const ros::Time& time, const ros::Duration& period)
  {

  }

  void ShipController::stopping(const ros::Time& time)
  {

  }

  bool ShipController::getMotorNames(ros::NodeHandle& controller_nh,const std::string& motor_param,std::vector<std::string>& motor_names)
  {
    XmlRpc::XmlRpcValue motor_list;
    if (!controller_nh.getParam(motor_param, motor_list))
    {
      ROS_ERROR_STREAM_NAMED(name,
          "Couldn't retrieve wheel param '" << motor_param << "'.");
      return false;
    }
    if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      if (motor_list.size() == 0)
      {
        ROS_ERROR_STREAM_NAMED(name,
            "Wheel param '" << motor_param << "' is an empty list");
        return false;
      }
      for (int i = 0; i < motor_list.size(); ++i)
      {
        if (motor_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR_STREAM_NAMED(name,
              "Wheel param '" << motor_param << "' #" << i <<
              " isn't a string.");
          return false;
        }
      }
      motor_names.resize(motor_list.size());
      for (int i = 0; i < motor_list.size(); ++i)
      {
        motor_names[i] = static_cast<std::string>(motor_list[i]);
      }
    }
    else if (motor_list.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      motor_names.push_back(motor_list);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(name,
          "Wheel param '" << motor_param <<
          "' is neither a list of strings nor a string.");
      return false;
    }
    return true;
  }
}
