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

#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CpOffboardKeepAlive::CpOffboardKeepAlive() {}

CpOffboardKeepAlive::~CpOffboardKeepAlive() {}

void CpOffboardKeepAlive::onInitialize()
{
  auto node = this->getNode();
  publisher_ = node->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", rclcpp::QoS(1).best_effort());

  this->requiresComponent(trajectorySetpoint_);

  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: initialized (disabled)");
}

void CpOffboardKeepAlive::update()
{
  if (!enabled_) return;

  auto node = this->getNode();

  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  publisher_->publish(msg);

  // Republish the latest trajectory setpoint to keep PX4 happy
  if (trajectorySetpoint_)
  {
    trajectorySetpoint_->republishLast();
  }
}

void CpOffboardKeepAlive::enable()
{
  enabled_ = true;
  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: ENABLED");
}

void CpOffboardKeepAlive::disable()
{
  enabled_ = false;
  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: DISABLED");
}

bool CpOffboardKeepAlive::isEnabled() const { return enabled_; }

}  // namespace cl_px4_mr
