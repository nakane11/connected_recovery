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
#include <expand_padding/expand_padding.h>
#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <string>
// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(expand_padding::ExpandPadding, nav_core::RecoveryBehavior)

namespace expand_padding
{
  ExpandPadding::ExpandPadding(): initialized_(false)
{
}

void ExpandPadding::initialize(std::string name, tf2_ros::Buffer*,
                               costmap_2d::Costmap2DROS* global_costmap,
                               costmap_2d::Costmap2DROS* local_costmap)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!initialized_)
  {
    ros::NodeHandle pnh_("~/" + name);
    reconfigure_name_ = pnh_.param<std::string>("reconfigure_name", "/move_base_node/global_costmap");
    footprint_padding_ = pnh_.param("footprint_padding", 0.3);
    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }

}

ExpandPadding::~ExpandPadding()
{
}

void ExpandPadding::runBehavior()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }
  ROS_WARN("Expand Padding recovery behavior started.");
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;
  double_param.name = "footprint_padding";
  double_param.value = footprint_padding_;
  conf.doubles.push_back(double_param);
  srv_req.config = conf;
  std::string s = reconfigure_name_ + "/set_parameters";
  if (ros::service::call(s, srv_req, srv_resp)) {
    ROS_INFO("call to set global_costmap parameters succeeded");
  } else {
    ROS_INFO("call to set global_costmap parameters failed");
  }
}
};  // namespace expand_padding
