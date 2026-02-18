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

#include <cmath>
#include <mutex>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleLocalPosition;

class CpTrajectorySetpoint : public smacc2::ISmaccComponent
{
public:
  CpTrajectorySetpoint();
  virtual ~CpTrajectorySetpoint();

  void onInitialize() override;

  void setPositionNED(
    float x, float y, float z, float yaw = std::numeric_limits<float>::quiet_NaN());
  void hold();
  void republishLast();

  px4_msgs::msg::TrajectorySetpoint getLastSetpoint() const;

private:
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
  CpVehicleLocalPosition * localPosition_ = nullptr;
  px4_msgs::msg::TrajectorySetpoint lastSetpoint_;
  mutable std::mutex mutex_;
  bool hasPublished_ = false;
};

}  // namespace cl_px4_mr
