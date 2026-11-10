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

#include <sm_coretest_post_during_exit_1/events/ev_pulse.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_post_during_exit_1
{
// Posts EvPulse at pulse_rate_hz from a node wall timer, i.e. from the executor thread.
class CpEventPulse : public smacc2::ISmaccComponent
{
public:
  void onInitialize() override
  {
    auto node = getNode();
    node->declare_parameter<double>("pulse_rate_hz", 200.0);
    double rateHz = 200.0;
    node->get_parameter("pulse_rate_hz", rateHz);
    RCLCPP_INFO(getLogger(), "[CpEventPulse] posting EvPulse at %.1f Hz", rateHz);
    timer_ = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / rateHz),
      [this]()
      {
        pulses_++;
        this->postEvent<EvPulse>();
      });
  }

  uint32_t pulses() const { return pulses_; }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint32_t> pulses_{0};
};
}  // namespace sm_coretest_post_during_exit_1
