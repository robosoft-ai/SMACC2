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

#include "sm_autoware_avp/clients/autoware_client/client_behaviors/cb_setup_initial_pose_estimation.hpp"
#include "sm_autoware_avp/clients/autoware_client/cl_autoware.hpp"

namespace sm_autoware_avp
{
namespace clients
{
namespace autoware_client
{
CbSetupInitialPoseEstimation::CbSetupInitialPoseEstimation(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initialStatePose)
: initialStatePose_(initialStatePose)
{
}

void CbSetupInitialPoseEstimation::onEntry()
{
  sm_autoware_avp::clients::ClAutoware * autowareClient_;
  initialStatePose_.header.stamp = getNode()->now();
  initialStatePose_.header.frame_id = "map";

  this->requiresClient(autowareClient_);

  autowareClient_->publishInitialPoseEstimation(initialStatePose_);
}

void CbSetupInitialPoseEstimation::onExit() {}

}  // namespace autoware_client
}  // namespace clients
}  // namespace sm_autoware_avp
