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

#include <cl_px4_mr/client_behaviors/cb_takeoff.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

#include <chrono>
#include <thread>

namespace cl_px4_mr
{

CbTakeOff::CbTakeOff(float targetAltitude) : targetAltitude_(targetAltitude) {}

void CbTakeOff::onEntry()
{
  this->requiresComponent(vehicleCommand_);
  this->requiresComponent(offboardKeepAlive_);
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(goalChecker_);
  this->requiresComponent(localPosition_);

  this->getStateMachine()->createSignalConnection(
    goalChecker_->onGoalReached_, &CbTakeOff::onGoalReachedCallback, this);

  // 1. Enable offboard keepalive heartbeat
  offboardKeepAlive_->enable();

  // 2. Hold current position to begin streaming setpoints
  trajectorySetpoint_->hold();

  // 3. Wait for PX4 to see the setpoint stream (needs >2Hz for 1 second)
  RCLCPP_INFO(getLogger(), "CbTakeOff: streaming setpoints before offboard switch...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 4. Switch to offboard mode
  RCLCPP_INFO(getLogger(), "CbTakeOff: switching to offboard mode");
  vehicleCommand_->setOffboardMode();

  // 5. Small delay for mode switch to take effect
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // 6. Command target altitude (NED: up is negative Z)
  float currentX = localPosition_->getX();
  float currentY = localPosition_->getY();
  float targetZ = -targetAltitude_;
  float currentHeading = localPosition_->getHeading();

  RCLCPP_INFO(
    getLogger(), "CbTakeOff: commanding altitude %.2f m (NED z=%.2f)", targetAltitude_, targetZ);
  trajectorySetpoint_->setPositionNED(currentX, currentY, targetZ, currentHeading);

  // 7. Set goal checker for target altitude
  goalChecker_->setGoal(currentX, currentY, targetZ, 0.5f, 0.3f);
}

void CbTakeOff::onExit() {}

void CbTakeOff::onGoalReachedCallback()
{
  RCLCPP_INFO(getLogger(), "CbTakeOff: target altitude reached - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
