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

#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
// waits a robot pose message. Usually used for the startup synchronization.
enum class WaitPoseStandardReferenceFrame
{
  Map,
  Odometry
};

// Waits a new pose using the Pose Component
// the specific pose to wait is configured in that component
class CbWaitPose : public smacc2::SmaccAsyncClientBehavior
{
public:
  // waits a new pose update of the Pose Component
  CbWaitPose();

  // waits a new pose update of the Pose Component in some reference frame (if there is no connection it will wait)
  CbWaitPose(WaitPoseStandardReferenceFrame frame);

  virtual ~CbWaitPose();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  void onEntry() override;

protected:
  cl_nav2z::ClNav2Z * moveBaseClient_;
};
}  // namespace cl_nav2z
