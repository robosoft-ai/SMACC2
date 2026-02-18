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

#include <cl_px4_mr/components/cp_vehicle_command.hpp>

namespace cl_px4_mr
{

CpVehicleCommand::CpVehicleCommand() {}

CpVehicleCommand::~CpVehicleCommand() {}

void CpVehicleCommand::onInitialize()
{
  auto node = this->getNode();
  publisher_ = node->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", rclcpp::QoS(1).best_effort());
  RCLCPP_INFO(getLogger(), "CpVehicleCommand: publisher created on /fmu/in/vehicle_command");
}

void CpVehicleCommand::sendCommand(
  uint32_t command, float param1, float param2, float param3, float param4, double param5,
  double param6, float param7)
{
  auto node = this->getNode();
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  publisher_->publish(msg);
  RCLCPP_INFO(
    getLogger(), "CpVehicleCommand: sent command %u (p1=%.1f p2=%.1f p7=%.1f)", command, param1,
    param2, param7);
}

void CpVehicleCommand::arm()
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void CpVehicleCommand::forceArm()
{
  // param2 = 21196 bypasses pre-arm checks (useful for SITL without RC/GCS)
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 21196.0f);
}

void CpVehicleCommand::disarm()
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void CpVehicleCommand::setOffboardMode()
{
  sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void CpVehicleCommand::land() { sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); }

void CpVehicleCommand::takeoff(float altitude)
{
  sendCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0f, 0.0f, 0.0f, 0.0f, 0.0, 0.0,
    altitude);
}

}  // namespace cl_px4_mr
