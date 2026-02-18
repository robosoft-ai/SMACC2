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

#include <cl_px4_mr/components/cp_vehicle_command_ack.hpp>

namespace cl_px4_mr
{

CpVehicleCommandAck::CpVehicleCommandAck() {}

CpVehicleCommandAck::~CpVehicleCommandAck() {}

void CpVehicleCommandAck::onInitialize()
{
  auto node = this->getNode();
  subscriber_ = node->create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
    std::bind(&CpVehicleCommandAck::onAckMessage, this, std::placeholders::_1));
  RCLCPP_INFO(getLogger(), "CpVehicleCommandAck: subscribed to /fmu/out/vehicle_command_ack");
}

void CpVehicleCommandAck::onAckMessage(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  lastCommand_ = msg->command;
  lastResult_ = msg->result;
  RCLCPP_INFO(getLogger(), "CpVehicleCommandAck: command=%u result=%u", msg->command, msg->result);
  onAckReceived_();
}

uint32_t CpVehicleCommandAck::getLastAckCommand() const { return lastCommand_; }

uint8_t CpVehicleCommandAck::getLastAckResult() const { return lastResult_; }

}  // namespace cl_px4_mr
