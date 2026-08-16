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

#include <cl_px4_mr/client_behaviors/cb_arm_px4.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>

#include <chrono>
#include <thread>

namespace cl_px4_mr
{

CbArmPX4::CbArmPX4() {}

void CbArmPX4::onEntry()
{
  // Enable offboard keepalive and set offboard mode so PX4's offboard signal
  // requirement is satisfied before arming. Without this, canArm() fails because
  // offboard_control_signal_lost is true when nav_state == OFFBOARD.
  if (!offboardKeepAlive_->isEnabled())
  {
    RCLCPP_INFO(getLogger(), "CbArmPX4: enabling offboard keepalive");
    offboardKeepAlive_->enable();
  }

  RCLCPP_INFO(getLogger(), "CbArmPX4: sending setOffboardMode command");
  vehicleCommand_->setOffboardMode();

  // Wait for PX4 to register offboard signal (needs at least one
  // offboard_control_mode message processed by PX4's health checks)
  RCLCPP_INFO(getLogger(), "CbArmPX4: waiting 2s for offboard signal registration...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (int attempt = 0; attempt < MAX_RETRIES; attempt++)
  {
    if (attempt < 2)
    {
      RCLCPP_INFO(
        getLogger(), "CbArmPX4: sending arm command (attempt %d/%d)", attempt + 1, MAX_RETRIES);
      vehicleCommand_->arm();
    }
    else
    {
      RCLCPP_WARN(getLogger(), "CbArmPX4: force-arming (attempt %d/%d)", attempt + 1, MAX_RETRIES);
      vehicleCommand_->forceArm();
    }

    // Wait for armed confirmation or timeout
    for (int i = 0; i < RETRY_INTERVAL_SEC * 10; i++)
    {
      if (armed_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (armed_)
    {
      RCLCPP_INFO(getLogger(), "CbArmPX4: vehicle ARMED - posting success");
      this->postPx4Success();
      return;
    }

    RCLCPP_WARN(
      getLogger(), "CbArmPX4: attempt %d/%d timed out, retrying...", attempt + 1, MAX_RETRIES);
  }

  RCLCPP_ERROR(getLogger(), "CbArmPX4: all %d attempts failed - posting failure", MAX_RETRIES);
  this->postPx4Failure();
}

void CbArmPX4::onExit() {}

void CbArmPX4::onArmedCallback() { armed_ = true; }

void CbArmPX4::wireCompletionSignals()
{
  if (vehicleStatus_ != nullptr)
  {
    this->getStateMachine()->createSignalConnection(
      vehicleStatus_->onArmed_, &CbArmPX4::onArmedCallback, this);
  }
  else
  {
    RCLCPP_WARN(getLogger(), "CbArmPX4: completion component missing, no completion signal wired");
  }
}

}  // namespace cl_px4_mr
