// Copyright 2026 RobosoftAI Inc.
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

#include <atomic>
#include <chrono>

#include <sm_coretest_reactor_order_1/events/ev_x.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_reactor_order_1
{
// Posts EvX every pulse_period_ms from a node wall timer, i.e. from the executor thread.
class CpEventPulse : public smacc2::ISmaccComponent
{
public:
  void onInitialize() override
  {
    auto node = getNode();
    node->declare_parameter<int>("pulse_period_ms", 500);
    int periodMs = 500;
    node->get_parameter("pulse_period_ms", periodMs);
    RCLCPP_INFO(getLogger(), "[CpEventPulse] posting EvX every %d ms", periodMs);
    timer_ = node->create_wall_timer(
      std::chrono::milliseconds(periodMs),
      [this]()
      {
        pulses_++;
        RCLCPP_INFO(getLogger(), "[CpEventPulse] pulse %u: posting EvX", pulses_.load());
        this->postEvent<EvX>();
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint32_t> pulses_{0};
};
}  // namespace sm_coretest_reactor_order_1
