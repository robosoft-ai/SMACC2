// Copyright 2021 RobosoftAI Inc.
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

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <smacc2/smacc.hpp>

namespace cl_ros2_timer
{
class CbTimerCountdownLoop : public smacc2::SmaccClientBehavior
{
public:
  explicit CbTimerCountdownLoop(int64_t triggerTickCount)
  : tickTriggerCount_(triggerTickCount), tickCounter_(0)
  {
  }

  void onEntry() override
  {
    this->requiresClient(timerClient_);

    // Get the core timer component
    smacc2::client_core_components::CpRos2Timer * timerComponent;
    this->requiresComponent(timerComponent);

    // Connect to the core timer component
    timerComponent->onTimerTick(&CbTimerCountdownLoop::onClientTimerTickCallback, this);
  }

  void onExit() override {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->postCountDownEvent_ = [=]()
    { this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>(); };
  }

  template <typename T>
  boost::signals2::connection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

private:
  int64_t tickTriggerCount_;
  int64_t tickCounter_;

  ClRos2Timer * timerClient_;
  std::function<void()> postCountDownEvent_;
  smacc2::SmaccSignal<void()> onTimerTick_;
  void onClientTimerTickCallback()
  {
    tickCounter_++;

    if (tickCounter_ % tickTriggerCount_ == 0)
    {
      onTimerTick_();
      postCountDownEvent_();
    }
  }
};
}  // namespace cl_ros2_timer
