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
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{
// Owns the subscription to PX4's failsafe flags for the life of the state machine and keeps the
// latest values; behaviors poll it instead of subscribing themselves (see CbConnectMicroRosAgent).
class CpFailsafeFlags : public smacc2::ISmaccComponent
{
public:
  CpFailsafeFlags();
  virtual ~CpFailsafeFlags();

  void onInitialize() override;

  // true once a message has been received
  bool hasMessage() const;
  bool isAttitudeInvalid() const;
  bool isLocalAltitudeInvalid() const;
  bool isLocalPositionInvalid() const;
  // a message has arrived and none of the three estimate flags is set
  bool isHealthOk() const;

private:
  void onFailsafeMessage(const px4_msgs::msg::FailsafeFlags::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr subscriber_;
  bool hasMessage_ = false;
  bool attitudeInvalid_ = true;
  bool localAltitudeInvalid_ = true;
  bool localPositionInvalid_ = true;
  mutable std::mutex mutex_;
};
}  // namespace cl_px4_mr
