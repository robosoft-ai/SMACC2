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

#include <smacc2/client_core_components/cp_ros2_timer.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace sm_coretest_signal_key_1
{
// DELIBERATELY declared with the non-SMACC base first. With this layout the
// ISmaccClientBehavior subobject is not at offset 0, so the address this behavior
// subscribes with ("this") differs from the address the orthogonal disposes it with
// (ISmaccClientBehavior*). The behavior subscribes to a component signal that keeps
// firing after the owning state exits.
class CbMixinFirstSubscriber : public smacc2::ISmaccUpdatable, public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    this->requiresComponent(tick_);
    tick_->onTimerTick(&CbMixinFirstSubscriber::onTick, this);
    RCLCPP_INFO(
      getLogger(), "[CbMixinFirstSubscriber] subscribed to onTimerTick; this=%p base=%p",
      static_cast<void *>(this),
      static_cast<void *>(static_cast<smacc2::ISmaccClientBehavior *>(this)));
  }

  void onExit() override
  {
    RCLCPP_INFO(getLogger(), "[CbMixinFirstSubscriber] onExit, ticks=%d", ticks_);
  }

  void update() override {}

private:
  void onTick()
  {
    ++ticks_;
    RCLCPP_INFO(getLogger(), "[CbMixinFirstSubscriber] TICK %d", ticks_);
  }

  smacc2::client_core_components::CpRos2Timer * tick_ = nullptr;
  int ticks_ = 0;
};
}  // namespace sm_coretest_signal_key_1
