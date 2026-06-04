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

  // Catch the old tick-count integer API and emit a migration hint at compile time.
  template <typename T, std::enable_if_t<std::is_integral<T>::value, int> = 0>
  explicit CbTimerCountdownOnce(T)
  {
    static_assert(
      !std::is_integral<T>::value,
      "\n\nCbTimerCountdownOnce no longer accepts a tick count.\n"
      "Pass a chrono duration literal instead:\n"
      "  configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);    // 5 seconds\n"
      "  configure_orthogonal<OrTimer, CbTimerCountdownOnce>(500ms); // 500 ms\n"
      "Migration guide: https://smacc2.robosoft.ai/how-to/how-to-cl-ros2-timer.html\n");
  }

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
