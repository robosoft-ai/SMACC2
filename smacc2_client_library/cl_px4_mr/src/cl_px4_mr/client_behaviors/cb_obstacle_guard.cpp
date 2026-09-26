// Copyright 2026 RobosoftAI Inc.
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
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>

namespace cl_px4_mr
{

CbObstacleGuard::CbObstacleGuard() {}

CbObstacleGuard::~CbObstacleGuard() {}

void CbObstacleGuard::onEntry()
{
  this->requiresComponent(guard_);
  if (guard_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(), "CbObstacleGuard: CpForwardObstacleGuard not found - obstacle events disabled");
    return;
  }
  // start from a clean slate: whatever the cone saw before this behavior
  // existed (the ground while on the pad, walls swept during a turn) is not a
  // detection; a real obstacle re-triggers within a couple of clouds
  guard_->reset();
  guard_->onObstacleTooClose(&CbObstacleGuard::onTooClose, this);
  guard_->onObstacleCleared(&CbObstacleGuard::onCleared, this);
  RCLCPP_INFO(getLogger(), "CbObstacleGuard: armed");
}

void CbObstacleGuard::onExit() { RCLCPP_INFO(getLogger(), "CbObstacleGuard: disarmed"); }

void CbObstacleGuard::onTooClose()
{
  if (postTooClose_)
  {
    postTooClose_();
  }
}

void CbObstacleGuard::onCleared()
{
  if (postCleared_)
  {
    postCleared_();
  }
}

}  // namespace cl_px4_mr
