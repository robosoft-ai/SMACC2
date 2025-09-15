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

#include <chrono>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/common.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

namespace smacc2
{
namespace client_core_components
{
using namespace smacc2::default_events;

class CpRos2Timer : public smacc2::ISmaccComponent
{
public:
  CpRos2Timer(rclcpp::Duration duration, bool oneshot = false);
  virtual ~CpRos2Timer();

  void onInitialize() override;

  smacc2::SmaccSignal<void()> onTimerTick_;

  void startTimer();
  void stopTimer();
  void cancelTimer();

  template <typename T>
  boost::signals2::connection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

private:
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration duration_;
  bool oneshot_;
  bool initialized_;
};

}  // namespace client_core_components
}  // namespace smacc2
