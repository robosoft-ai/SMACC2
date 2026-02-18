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

#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommandAck : public smacc2::ISmaccComponent
{
public:
  CpVehicleCommandAck();
  virtual ~CpVehicleCommandAck();

  void onInitialize() override;

  uint32_t getLastAckCommand() const;
  uint8_t getLastAckResult() const;

  smacc2::SmaccSignal<void()> onAckReceived_;

private:
  void onAckMessage(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscriber_;
  uint32_t lastCommand_ = 0;
  uint8_t lastResult_ = 0;
};

}  // namespace cl_px4_mr
