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

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <smacc2/smacc.hpp>

namespace cl_ros2_timer
{
class CbTimerCountdownOnce : public smacc2::SmaccClientBehavior
{
public:
  CbTimerCountdownOnce(rclcpp::Duration targetDuration) : targetDuration_(targetDuration) {}

  void onEntry() override
  {
    auto node = this->getNode();
    auto clock = node->get_clock();
    wallTimer_ = rclcpp::create_timer(
      node, clock, std::chrono::nanoseconds(targetDuration_.nanoseconds()),
      [this]()
      {
        wallTimer_->cancel();
        onTimerTick_();
        postCountDownEvent_();
      });
  }

  void onExit() override
  {
    if (wallTimer_)
    {
      wallTimer_->cancel();
      wallTimer_.reset();
    }
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->postCountDownEvent_ = [=]()
    { this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>(); };
  }

  template <typename T>
  smacc2::SmaccSignalConnection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

private:
  rclcpp::Duration targetDuration_;
  rclcpp::TimerBase::SharedPtr wallTimer_;

  std::function<void()> postCountDownEvent_;
  smacc2::SmaccSignal<void()> onTimerTick_;
};
}  // namespace cl_ros2_timer
