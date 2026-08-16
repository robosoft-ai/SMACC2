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

#include <cl_px4_mr/client_behaviors/cb_change_altitude.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CbChangeAltitude::CbChangeAltitude(float targetAltitude) : targetAltitude_(targetAltitude) {}

void CbChangeAltitude::onEntry()
{
  float currentX = localPosition_->getX();
  float currentY = localPosition_->getY();
  float currentHeading = localPosition_->getHeading();
  float targetZ = -targetAltitude_;  // NED: up is negative Z

  RCLCPP_INFO(
    getLogger(), "CbChangeAltitude: changing altitude to %.2f m (NED z=%.2f)", targetAltitude_,
    targetZ);

  trajectorySetpoint_->setPositionNED(currentX, currentY, targetZ, currentHeading);
  goalChecker_->setGoal(currentX, currentY, targetZ, 0.5f, 0.3f);
}

void CbChangeAltitude::onExit() { goalChecker_->clearGoal(); }

void CbChangeAltitude::onGoalReachedCallback()
{
  RCLCPP_INFO(getLogger(), "CbChangeAltitude: target altitude reached - posting success");
  this->postPx4Success();
}

void CbChangeAltitude::wireCompletionSignals()
{
  if (goalChecker_ != nullptr)
  {
    this->getStateMachine()->createSignalConnection(
      goalChecker_->onGoalReached_, &CbChangeAltitude::onGoalReachedCallback, this);
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "CbChangeAltitude: completion component missing, no completion signal wired");
  }
}

}  // namespace cl_px4_mr
