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

#include <ros_timer_client/cl_ros_timer.hpp>

#include <smacc2/smacc.hpp>

namespace cl_ros_timer
{
class CbTimerCountdownLoop : public smacc2::SmaccClientBehavior
{
public:
  explicit CbTimerCountdownLoop(unsigned long triggerTickCount);

  void onEntry() override;
  void onExit() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postCountDownEvent_ = [=]() {
      this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>();
    };
  }

  template <typename T>
  boost::signals2::connection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

private:
  unsigned long tickTriggerCount_;
  unsigned long tickCounter_;

  ClRosTimer * timerClient_;
  std::function<void()> postCountDownEvent_;
  smacc2::SmaccSignal<void()> onTimerTick_;
  void onClientTimerTickCallback();
};
}  // namespace cl_ros_timer
