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

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;

class CpOffboardKeepAlive : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpOffboardKeepAlive();
  virtual ~CpOffboardKeepAlive();

  void onInitialize() override;
  void update() override;

  void enable();
  void disable();
  bool isEnabled() const;

private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_;
  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  bool enabled_ = false;
};

}  // namespace cl_px4_mr
