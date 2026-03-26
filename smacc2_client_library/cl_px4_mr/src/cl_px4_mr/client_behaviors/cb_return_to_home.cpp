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

#include <cl_px4_mr/client_behaviors/cb_return_to_home.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CbReturnToHome::CbReturnToHome(float homeX, float homeY, float homeZ, float homeYaw)
: homeX_(homeX), homeY_(homeY), homeZ_(homeZ), homeYaw_(homeYaw)
{
}

void CbReturnToHome::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(goalChecker_);

  this->getStateMachine()->createSignalConnection(
    goalChecker_->onGoalReached_, &CbReturnToHome::onGoalReachedCallback, this);

  RCLCPP_INFO(
    getLogger(), "CbReturnToHome: returning to home [%.2f, %.2f, %.2f] yaw=%.2f", homeX_, homeY_,
    homeZ_, homeYaw_);

  trajectorySetpoint_->setPositionNED(homeX_, homeY_, homeZ_, homeYaw_);
  goalChecker_->setGoal(homeX_, homeY_, homeZ_);
}

void CbReturnToHome::onExit() { goalChecker_->clearGoal(); }

void CbReturnToHome::onGoalReachedCallback()
{
  RCLCPP_INFO(getLogger(), "CbReturnToHome: home position reached - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
