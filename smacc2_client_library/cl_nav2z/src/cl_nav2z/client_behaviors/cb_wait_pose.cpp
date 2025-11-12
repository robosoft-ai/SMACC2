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

#include <cl_nav2z/client_behaviors/cb_wait_pose.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

#include <cl_nav2z/common.hpp>
#include <rclcpp/parameter_client.hpp>

namespace cl_nav2z
{
using namespace smacc2;

CbWaitPose::CbWaitPose() {}

CbWaitPose::~CbWaitPose() {}

void CbWaitPose::onEntry()
{
  cl_nav2z::CpPose * pose = nullptr;
  this->requiresComponent(pose, ComponentRequirement::SOFT);
  try
  {
    pose->waitTransformUpdate(rclcpp::Rate(20));
    auto posemsg = pose->toPoseMsg();
    RCLCPP_INFO_STREAM(getLogger(), "[CbWaitPose] pose arrived: " << std::endl << posemsg);
  }
  catch (std::exception & ex)
  {
    RCLCPP_INFO(getLogger(), "[CbWaitPose] error getting the robot pose");
    this->postFailureEvent();
    return;
  }

  RCLCPP_INFO(getLogger(), "[CbWaitPose] pose received");
  this->postSuccessEvent();
}
}  // namespace cl_nav2z
