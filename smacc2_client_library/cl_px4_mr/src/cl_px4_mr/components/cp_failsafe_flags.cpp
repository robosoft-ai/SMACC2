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

#include <cl_px4_mr/components/cp_failsafe_flags.hpp>

namespace cl_px4_mr
{
CpFailsafeFlags::CpFailsafeFlags() {}

CpFailsafeFlags::~CpFailsafeFlags() {}

void CpFailsafeFlags::onInitialize()
{
  subscriber_ = getNode()->create_subscription<px4_msgs::msg::FailsafeFlags>(
    "/fmu/out/failsafe_flags", rclcpp::SensorDataQoS(),
    std::bind(&CpFailsafeFlags::onFailsafeMessage, this, std::placeholders::_1));
  RCLCPP_INFO(getLogger(), "CpFailsafeFlags: subscribed to /fmu/out/failsafe_flags");
}

void CpFailsafeFlags::onFailsafeMessage(const px4_msgs::msg::FailsafeFlags::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hasMessage_ = true;
  attitudeInvalid_ = msg->attitude_invalid;
  localAltitudeInvalid_ = msg->local_altitude_invalid;
  localPositionInvalid_ = msg->local_position_invalid;
}

bool CpFailsafeFlags::hasMessage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return hasMessage_;
}

bool CpFailsafeFlags::isAttitudeInvalid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return attitudeInvalid_;
}

bool CpFailsafeFlags::isLocalAltitudeInvalid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return localAltitudeInvalid_;
}

bool CpFailsafeFlags::isLocalPositionInvalid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return localPositionInvalid_;
}

bool CpFailsafeFlags::isHealthOk() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return hasMessage_ && !attitudeInvalid_ && !localAltitudeInvalid_ && !localPositionInvalid_;
}
}  // namespace cl_px4_mr
