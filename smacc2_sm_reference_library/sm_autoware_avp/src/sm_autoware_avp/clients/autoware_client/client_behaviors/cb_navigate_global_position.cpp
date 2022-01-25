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

#include "sm_autoware_avp/clients/autoware_client/client_behaviors/cb_navigate_global_position.hpp"
#include "sm_autoware_avp/clients/autoware_client/cl_autoware.hpp"

namespace sm_autoware_avp
{
namespace clients
{
namespace autoware_client
{

CbNavigateGlobalPosition::CbNavigateGlobalPosition(const geometry_msgs::msg::PoseStamped& goalPose)
: goalPose_(goalPose)
{

}

// CbNavigateGlobalPosition::CbNavigateGlobalPosition(float x, float y, float z, float yaw /*radians*/)
// {
//   goal_.x = x;
//   goal_.y = y;
//   goal_.z = z;
//   yaw_ = yaw;
// }

void CbNavigateGlobalPosition::onEntry()
{
  RCLCPP_INFO(getLogger(), "On Entry!");
  sm_autoware_avp::clients::ClAutoware * autowareClient_;

  this->requiresClient(autowareClient_);

  this->goalPose_.header.stamp = getNode()->now();

  autowareClient_->publishGoalPose(this->goalPose_);
}

void CbNavigateGlobalPosition::onExit() {}
}  // namespace autoware_client

}  // namespace clients
}  // namespace sm_autoware_avp
