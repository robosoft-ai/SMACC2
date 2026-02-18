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

#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand : public smacc2::ISmaccComponent
{
public:
  CpVehicleCommand();
  virtual ~CpVehicleCommand();

  void onInitialize() override;

  void sendCommand(
    uint32_t command, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f,
    float param4 = 0.0f, double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);

  void arm();
  void forceArm();
  void disarm();
  void setOffboardMode();
  void land();
  void takeoff(float altitude);

private:
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;
};

}  // namespace cl_px4_mr
