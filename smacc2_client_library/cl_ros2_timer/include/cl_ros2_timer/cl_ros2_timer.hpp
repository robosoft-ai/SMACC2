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

#include <boost/signals2.hpp>
#include <chrono>
#include <optional>
#include <smacc2/smacc.hpp>

#include <cl_ros2_timer/components/cp_timer_listener_1.hpp>
#include <smacc2/client_core_components/cp_ros2_timer.hpp>

namespace cl_ros2_timer
{
template <typename TSource, typename TOrthogonal>
struct EvTimer : sc::event<EvTimer<TSource, TOrthogonal>>
{
};

class ClRos2Timer : public smacc2::ISmaccClient
{
public:
  ClRos2Timer(rclcpp::Duration duration, bool oneshot = false);

  virtual ~ClRos2Timer();

  // Component-based initialization
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Create the core timer component
    this->createComponent<smacc2::client_core_components::CpRos2Timer, TOrthogonal, ClRos2Timer>(
      duration_, oneshot_);

    // Create the timer listener component that requires CpRos2Timer
    this->createComponent<cl_ros2_timer::components::CpTimerListener1, TOrthogonal, ClRos2Timer>();
  }

private:
  rclcpp::Duration duration_;
  bool oneshot_;
};
}  // namespace cl_ros2_timer
