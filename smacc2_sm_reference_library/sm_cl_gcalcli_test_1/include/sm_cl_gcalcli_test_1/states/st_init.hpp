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
struct StInit : smacc2::SmaccState<StInit, SmClGcalcliTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Timer triggers transition to next state after 1 second
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StWaitConnection, SUCCESS>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Quick timer to move to the test states
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(1s);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "sm_cl_gcalcli_test_1 - StInit");
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "Starting cl_gcalcli unit tests...");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "StInit - onExit()"); }
};

}  // namespace sm_cl_gcalcli_test_1
