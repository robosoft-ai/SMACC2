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

#include <smacc2/client_core_components/cp_ros2_timer.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace sm_coretest_object_churn_1
{
// A synchronous behavior that is also an updatable and holds a signal connection for its lifetime.
class CbTickSubscriber : public smacc2::SmaccClientBehavior, public smacc2::ISmaccUpdatable
{
public:
  void onEntry() override
  {
    this->requiresComponent(tick_);
    tick_->onTimerTick(&CbTickSubscriber::onTick, this);
  }

  void update() override { ++updates_; }

private:
  void onTick() { ++ticks_; }

  smacc2::client_core_components::CpRos2Timer * tick_ = nullptr;
  unsigned long ticks_ = 0;
  unsigned long updates_ = 0;
};
}  // namespace sm_coretest_object_churn_1
