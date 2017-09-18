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
 * Author: Enrique Fernández
 */

#ifndef SHIP_CONTROLLER_H_INCLUDED
#define SHIP_CONTROLLER_H_INCLUDED

//headers in this packages
#include <speed_limiter.h>

//headers for controller_interface
#include <controller_interface/controller.h>

//headers for fardware_interface
#include <hardware_interface/joint_command_interface.h>

//headers for pluginlib
#include <pluginlib/class_list_macros.h>

//headers for realtime_tools
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

//headers for boost
#include <boost/shared_ptr.hpp>

namespace ship_controller
{
  class ShipController: public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    ShipController();

    virtual bool init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping(const ros::Time& time);

  private:
    hardware_interface::JointHandle left_motor_joint;
    hardware_interface::JointHandle right_motor_joint;
    ros::Subscriber sub_command;
    struct Commands
    {
      double lin;
      double ang;
      ros::Time stamp;

      Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command;
    Commands command_struct;
    SpeedLimiter limiter_lin,limiter_ang;
    size_t motor_joints_size;
  };
  PLUGINLIB_EXPORT_CLASS(ship_controller::ShipController, controller_interface::ControllerBase);
}
#endif
