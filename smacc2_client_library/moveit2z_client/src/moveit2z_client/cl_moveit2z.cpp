// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <moveit2z_client/cl_moveit2z.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace moveit::planning_interface;

namespace cl_moveit2z
{
ClMoveit2z::ClMoveit2z(const moveit::planning_interface::MoveGroupInterface::Options & options)
: options_(options)
{
}

ClMoveit2z::ClMoveit2z(std::string groupName) : options_(groupName) {}

ClMoveit2z::~ClMoveit2z() {}

void ClMoveit2z::onInitialize()
{
  moveGroupClientInterface = std::make_shared<MoveGroupInterface>(getNode(), options_);
  planningSceneInterface = std::make_shared<PlanningSceneInterface>(options_.move_group_namespace_);
}

void ClMoveit2z::postEventMotionExecutionSucceded()
{
  RCLCPP_INFO(getLogger(), "[ClMoveit2z] Post Motion Success Event");
  postEventMotionExecutionSucceded_();
}

void ClMoveit2z::postEventMotionExecutionFailed()
{
  RCLCPP_INFO(getLogger(), "[ClMoveit2z] Post Motion Failure Event");
  postEventMotionExecutionFailed_();
}

const moveit::planning_interface::MoveGroupInterface::Options & ClMoveit2z::getOptions() const
{
  return options_;
}

}  // namespace cl_moveit2z
