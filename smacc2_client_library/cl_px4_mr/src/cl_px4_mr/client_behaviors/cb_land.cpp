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

#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>

namespace cl_px4_mr
{

CbLand::CbLand() {}

void CbLand::onEntry()
{
  this->requiresComponent(vehicleCommand_);
  this->requiresComponent(vehicleStatus_);
  this->requiresComponent(offboardKeepAlive_);

  // Connect to disarmed signal - PX4 auto-disarms after landing
  this->getStateMachine()->createSignalConnection(
    vehicleStatus_->onDisarmed_, &CbLand::onLandedCallback, this);

  // Disable offboard keepalive - land command uses its own mode
  offboardKeepAlive_->disable();

  RCLCPP_INFO(getLogger(), "CbLand: sending land command");
  vehicleCommand_->land();
}

void CbLand::onExit() {}

void CbLand::onLandedCallback()
{
  RCLCPP_INFO(getLogger(), "CbLand: vehicle landed and disarmed - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
