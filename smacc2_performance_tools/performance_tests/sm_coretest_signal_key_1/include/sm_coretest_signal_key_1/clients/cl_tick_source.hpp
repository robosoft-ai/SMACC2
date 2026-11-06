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

namespace sm_coretest_signal_key_1
{
// A client whose only job is to own a free-running CpRos2Timer. The component lives for the
// whole state machine, so its onTimerTick signal keeps firing after any state has exited.
class ClTickSource : public smacc2::ISmaccClient
{
public:
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    this->createComponent<smacc2::client_core_components::CpRos2Timer, TOrthogonal, TClient>(
      rclcpp::Duration(std::chrono::milliseconds(200)));
  }
};
}  // namespace sm_coretest_signal_key_1
