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

#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CbGoToLocation::CbGoToLocation(float targetX, float targetY, float targetZ, float yaw)
: targetX_(targetX), targetY_(targetY), targetZ_(targetZ), yaw_(yaw)
{
}

void CbGoToLocation::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(goalChecker_);

  this->getStateMachine()->createSignalConnection(
    goalChecker_->onGoalReached_, &CbGoToLocation::onGoalReachedCallback, this);

  RCLCPP_INFO(
    getLogger(), "CbGoToLocation: navigating to [%.2f, %.2f, %.2f] yaw=%.2f", targetX_, targetY_,
    targetZ_, yaw_);

  trajectorySetpoint_->setPositionNED(targetX_, targetY_, targetZ_, yaw_);
  goalChecker_->setGoal(targetX_, targetY_, targetZ_);
}

void CbGoToLocation::onExit() { goalChecker_->clearGoal(); }

void CbGoToLocation::onGoalReachedCallback()
{
  RCLCPP_INFO(getLogger(), "CbGoToLocation: goal reached - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
