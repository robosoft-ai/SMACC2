// Copyright 2024 RobosoftAI Inc.
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

#include <smacc2/smacc.hpp>

namespace sm_cl_gcalcli_test_1
{

using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StDone : smacc2::SmaccState<StDone, SmClGcalcliTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Loop back to start for continuous testing
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StInit, SUCCESS>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Wait 5 seconds before looping
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "StDone - Test Cycle Complete");
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "All cl_gcalcli behavior tests completed.");
    RCLCPP_INFO(getLogger(), "Looping back to StInit in 5 seconds...");
    RCLCPP_INFO(getLogger(), "(Press Ctrl+C to exit)");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "StDone - onExit()"); }
};

}  // namespace sm_cl_gcalcli_test_1
