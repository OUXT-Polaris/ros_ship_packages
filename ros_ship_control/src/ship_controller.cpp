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
    std::string left_motor_name,right_motor_name;
    if(! getMotorName(controller_nh, "left_motor", left_motor_name) or ! getMotorName(controller_nh, "right_motor", right_motor_name))
    {
      return false;
    }
    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"      , limiter_lin.has_velocity_limits     , limiter_lin.has_velocity_limits     );
    controller_nh.param("linear/x/has_acceleration_limits"  , limiter_lin.has_acceleration_limits , limiter_lin.has_acceleration_limits );
    controller_nh.param("linear/x/has_jerk_limits"          , limiter_lin.has_jerk_limits         , limiter_lin.has_jerk_limits         );
    controller_nh.param("linear/x/max_velocity"             , limiter_lin.max_velocity            ,  limiter_lin.max_velocity           );
    controller_nh.param("linear/x/min_velocity"             , limiter_lin.min_velocity            , -limiter_lin.max_velocity           );
    controller_nh.param("linear/x/max_acceleration"         , limiter_lin.max_acceleration        ,  limiter_lin.max_acceleration       );
    controller_nh.param("linear/x/min_acceleration"         , limiter_lin.min_acceleration        , -limiter_lin.max_acceleration       );
    controller_nh.param("linear/x/min_jerk"                 , limiter_lin.min_jerk                , -limiter_lin.max_jerk               );
    controller_nh.param("linear/x/max_jerk"                 , limiter_lin.max_jerk                ,  limiter_lin.max_jerk               );

    controller_nh.param("angular/z/has_velocity_limits"     , limiter_ang.has_velocity_limits    , limiter_ang.has_velocity_limits     );
    controller_nh.param("angular/z/has_acceleration_limits" , limiter_ang.has_acceleration_limits, limiter_ang.has_acceleration_limits );
    controller_nh.param("angular/z/has_jerk_limits"         , limiter_ang.has_jerk_limits        , limiter_ang.has_jerk_limits         );
    controller_nh.param("angular/z/max_velocity"            , limiter_ang.max_velocity           ,  limiter_ang.max_velocity           );
    controller_nh.param("angular/z/min_velocity"            , limiter_ang.min_velocity           , -limiter_ang.max_velocity           );
    controller_nh.param("angular/z/max_acceleration"        , limiter_ang.max_acceleration       ,  limiter_ang.max_acceleration       );
    controller_nh.param("angular/z/min_acceleration"        , limiter_ang.min_acceleration       , -limiter_ang.max_acceleration       );
    controller_nh.param("angular/z/max_jerk"                , limiter_ang.max_jerk               ,  limiter_ang.max_jerk               );
    controller_nh.param("angular/z/min_jerk"                , limiter_ang.min_jerk               , -limiter_ang.max_jerk               );
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

  bool ShipController::getMotorName(ros::NodeHandle& controller_nh,const std::string& motor_param,std::string& motor_name)
  {
    if (!controller_nh.getParam(motor_param, motor_name))
    {
      ROS_ERROR_STREAM_NAMED(name,"Couldn't retrieve wheel param '" << motor_param << "'.");
      return false;
    }
    return true;
  }
}
