/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <partial_rotate/partial_rotate.h>
#include <pluginlib/class_list_macros.h>
// #include <nav_core/parameter_magic.h>
// #include <tf2/utils.h>
#include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Point.h>
// #include <angles/angles.h>
// #include <algorithm>
#include <string>
#include <std_srvs/Empty.h>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(partial_rotate::PartialRotate, nav_core::RecoveryBehavior)

namespace partial_rotate
{
  PartialRotate::PartialRotate(): pnh_("~"), initialized_(false)
{
}

void PartialRotate::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS*)
{
  if (!initialized_)
  {
    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

PartialRotate::~PartialRotate()
{
}

void PartialRotate::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");
  client = pnh_.serviceClient<std_srvs::Empty>("/partial_rotate");
  std_srvs::Empty srv;
  if (client.waitForExistence(ros::Duration(1.0)))
    {
      if (client.call(srv))
        {
          ROS_INFO("Succeed in calling service partial_rotate");
        }
      else
        {
          ROS_ERROR("Failed to call service partial_rotate");
        }
    }
  else
    {
      ROS_ERROR("Failed to find service partial_rotate");
    }
}
};  // namespace partial_rotate
