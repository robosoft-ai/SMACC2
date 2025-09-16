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

#include <smacc2/client_core_components/cp_ros2_timer.hpp>
#include <smacc2/smacc.hpp>

namespace cl_ros2_timer
{
// Forward declaration for EvTimer - defined in cl_ros2_timer.hpp
template <typename TSource, typename TOrthogonal>
struct EvTimer;

namespace components
{

class CpTimerListener1 : public smacc2::ISmaccComponent
{
public:
  CpTimerListener1() {}

  virtual ~CpTimerListener1() {}

  template <typename TOrthogonal, typename TClient>
  void onStateOrthogonalAllocation()
  {
    RCLCPP_INFO(getLogger(), "CpTimerListener1 onStateOrthogonalAllocation.");

    this->postTimerEvent_ = [this]()
    { this->template postEvent<EvTimer<CpTimerListener1, TOrthogonal>>(); };
  }

  void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "CpTimerListener1 initialization.");

    // Require the CpRos2Timer component (similar to CpKeyboardListener1 requiring CpTopicSubscriber)
    this->requiresComponent(timerComponent_);

    // Connect to the timer component's tick signal
    timerComponent_->onTimerTick(&CpTimerListener1::onTimerTickCallback, this);
  }

  smacc2::SmaccSignal<void()> onTimerCompleted_;

  template <typename T>
  boost::signals2::connection onTimerCompleted(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerCompleted_, callback, object);
  }

private:
  smacc2::client_core_components::CpRos2Timer * timerComponent_;
  std::function<void()> postTimerEvent_;

  void onTimerTickCallback()
  {
    RCLCPP_DEBUG(getLogger(), "CpTimerListener1 received timer tick.");

    // Emit our own signal for client behaviors to connect to
    if (!onTimerCompleted_.empty())
    {
      onTimerCompleted_();
    }

    // Post SMACC2 event for state transitions
    if (postTimerEvent_)
    {
      postTimerEvent_();
    }
  }
};

}  // namespace components
}  // namespace cl_ros2_timer
