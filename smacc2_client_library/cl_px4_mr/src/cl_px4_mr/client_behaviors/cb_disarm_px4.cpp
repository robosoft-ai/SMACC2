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

#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>

namespace cl_px4_mr
{

CbDisarmPX4::CbDisarmPX4() {}

void CbDisarmPX4::onEntry()
{
  RCLCPP_INFO(
    getLogger(), "CbDisarmPX4: sending disarm command (attempt %d/%d)", retryCount_ + 1,
    MAX_RETRIES);
  vehicleCommand_->disarm();
}

void CbDisarmPX4::onExit() {}

void CbDisarmPX4::onDisarmedCallback()
{
  RCLCPP_INFO(getLogger(), "CbDisarmPX4: vehicle DISARMED - posting success");
  this->postPx4Success();
}

void CbDisarmPX4::wireCompletionSignals()
{
  if (vehicleStatus_ != nullptr)
  {
    this->getStateMachine()->createSignalConnection(
      vehicleStatus_->onDisarmed_, &CbDisarmPX4::onDisarmedCallback, this);
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "CbDisarmPX4: completion component missing, no completion signal wired");
  }
}

}  // namespace cl_px4_mr
