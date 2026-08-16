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

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <atomic>

namespace cl_px4_mr
{

class CpMicroRosAgent;

class CbConnectMicroRosAgent : public CbPx4ClientBehaviorBase
{
public:
  CbConnectMicroRosAgent(double timeoutSec = 30.0);

  void onEntry() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(microRosAgent_, smacc2::ComponentRequirement::SOFT);
    CbPx4ClientBehaviorBase::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }
  void onExit() override;

private:
  CpMicroRosAgent * microRosAgent_ = nullptr;
  double timeoutSec_;
  rclcpp::Rate rate_;

  rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafeSub_;
  std::atomic<bool> healthOk_{false};
  std::atomic<bool> attitudeInvalid_{true};
  std::atomic<bool> localAltitudeInvalid_{true};
  std::atomic<bool> localPositionInvalid_{true};
};

}  // namespace cl_px4_mr
