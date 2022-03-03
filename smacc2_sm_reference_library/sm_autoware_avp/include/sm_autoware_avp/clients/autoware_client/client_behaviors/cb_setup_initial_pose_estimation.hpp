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
#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "smacc2/smacc_asynchronous_client_behavior.hpp"

namespace sm_autoware_avp
{
namespace clients
{
namespace autoware_client
{
class CbSetupInitialPoseEstimation : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbSetupInitialPoseEstimation(const geometry_msgs::msg::PoseWithCovarianceStamped & initialStatePose);

  virtual void onEntry() override;

  void onExit() override;

private:
  geometry_msgs::msg::PoseWithCovarianceStamped initialStatePose_;
};
}  // namespace autoware_client

}  // namespace clients
}  // namespace sm_autoware_avp
