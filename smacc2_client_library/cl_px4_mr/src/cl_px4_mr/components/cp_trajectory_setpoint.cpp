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

#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CpTrajectorySetpoint::CpTrajectorySetpoint() {}

CpTrajectorySetpoint::~CpTrajectorySetpoint() {}

void CpTrajectorySetpoint::onInitialize()
{
  auto node = this->getNode();
  publisher_ = node->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", rclcpp::QoS(1).best_effort());

  this->requiresComponent(localPosition_);

  // Initialize lastSetpoint_ with NaNs
  auto nan = std::numeric_limits<float>::quiet_NaN();
  lastSetpoint_.position = {nan, nan, nan};
  lastSetpoint_.velocity = {nan, nan, nan};
  lastSetpoint_.acceleration = {nan, nan, nan};
  lastSetpoint_.jerk = {nan, nan, nan};
  lastSetpoint_.yaw = nan;
  lastSetpoint_.yawspeed = nan;

  RCLCPP_INFO(
    getLogger(), "CpTrajectorySetpoint: publisher created on /fmu/in/trajectory_setpoint");
}

void CpTrajectorySetpoint::setPositionNED(float x, float y, float z, float yaw)
{
  auto node = this->getNode();
  auto nan = std::numeric_limits<float>::quiet_NaN();

  px4_msgs::msg::TrajectorySetpoint msg;
  msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  msg.position = {x, y, z};
  msg.velocity = {nan, nan, nan};
  msg.acceleration = {nan, nan, nan};
  msg.jerk = {nan, nan, nan};
  msg.yaw = yaw;
  msg.yawspeed = nan;

  publisher_->publish(msg);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    lastSetpoint_ = msg;
    hasPublished_ = true;
  }

  RCLCPP_INFO(
    getLogger(), "CpTrajectorySetpoint: position NED [%.2f, %.2f, %.2f] yaw=%.2f", x, y, z, yaw);
}

void CpTrajectorySetpoint::hold()
{
  if (!localPosition_ || !localPosition_->isValid())
  {
    RCLCPP_WARN(getLogger(), "CpTrajectorySetpoint::hold() - position not yet valid, skipping");
    return;
  }

  setPositionNED(
    localPosition_->getX(), localPosition_->getY(), localPosition_->getZ(),
    localPosition_->getHeading());
}

void CpTrajectorySetpoint::republishLast()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!hasPublished_) return;

  auto node = this->getNode();
  lastSetpoint_.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  publisher_->publish(lastSetpoint_);
}

px4_msgs::msg::TrajectorySetpoint CpTrajectorySetpoint::getLastSetpoint() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return lastSetpoint_;
}

}  // namespace cl_px4_mr
