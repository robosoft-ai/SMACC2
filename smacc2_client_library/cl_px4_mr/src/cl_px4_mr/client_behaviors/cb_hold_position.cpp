// Copyright 2025 Robosoft Inc.
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

#include <cl_px4_mr/client_behaviors/cb_hold_position.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CbHoldPosition::CbHoldPosition(float durationSeconds) : durationSeconds_(durationSeconds) {}

void CbHoldPosition::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);

  RCLCPP_INFO(getLogger(), "CbHoldPosition: holding position for %.1f seconds", durationSeconds_);

  trajectorySetpoint_->hold();
  startTime_ = std::chrono::steady_clock::now();
}

void CbHoldPosition::onExit() {}

void CbHoldPosition::update()
{
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - startTime_).count();

  if (elapsed >= durationSeconds_)
  {
    RCLCPP_INFO(
      getLogger(), "CbHoldPosition: duration reached (%.1f s) - posting success", elapsed);
    this->postSuccessEvent();
  }
}

}  // namespace cl_px4_mr
