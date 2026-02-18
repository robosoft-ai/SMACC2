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

#pragma once

#include <mutex>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleStatus : public smacc2::ISmaccComponent
{
public:
  CpVehicleStatus();
  virtual ~CpVehicleStatus();

  void onInitialize() override;

  bool isArmed() const;
  bool isLanded() const;
  uint8_t getNavState() const;
  uint8_t getArmingState() const;

  smacc2::SmaccSignal<void()> onArmed_;
  smacc2::SmaccSignal<void()> onDisarmed_;
  smacc2::SmaccSignal<void()> onModeChanged_;
  smacc2::SmaccSignal<void()> onLanded_;

private:
  void onStatusMessage(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscriber_;
  uint8_t armingState_ = 0;
  uint8_t navState_ = 0;
  uint8_t prevArmingState_ = 0;
  uint8_t prevNavState_ = 0;
  mutable std::mutex mutex_;
};

}  // namespace cl_px4_mr
