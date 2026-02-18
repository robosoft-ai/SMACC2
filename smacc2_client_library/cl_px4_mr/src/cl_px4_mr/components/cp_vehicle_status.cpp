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

#include <cl_px4_mr/components/cp_vehicle_status.hpp>

namespace cl_px4_mr
{

CpVehicleStatus::CpVehicleStatus() {}

CpVehicleStatus::~CpVehicleStatus() {}

void CpVehicleStatus::onInitialize()
{
  auto node = this->getNode();
  subscriber_ = node->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
    std::bind(&CpVehicleStatus::onStatusMessage, this, std::placeholders::_1));
  RCLCPP_INFO(getLogger(), "CpVehicleStatus: subscribed to /fmu/out/vehicle_status");
}

void CpVehicleStatus::onStatusMessage(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  prevArmingState_ = armingState_;
  prevNavState_ = navState_;
  armingState_ = msg->arming_state;
  navState_ = msg->nav_state;

  // Detect arming state changes
  if (armingState_ != prevArmingState_)
  {
    if (armingState_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      RCLCPP_INFO(getLogger(), "CpVehicleStatus: ARMED");
      onArmed_();
    }
    else if (armingState_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {
      RCLCPP_INFO(getLogger(), "CpVehicleStatus: DISARMED");
      onDisarmed_();
    }
  }

  // Detect nav state changes
  if (navState_ != prevNavState_)
  {
    RCLCPP_INFO(
      getLogger(), "CpVehicleStatus: nav_state changed %u -> %u", prevNavState_, navState_);
    onModeChanged_();
  }
}

bool CpVehicleStatus::isArmed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return armingState_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool CpVehicleStatus::isLanded() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return armingState_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
}

uint8_t CpVehicleStatus::getNavState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return navState_;
}

uint8_t CpVehicleStatus::getArmingState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return armingState_;
}

}  // namespace cl_px4_mr
