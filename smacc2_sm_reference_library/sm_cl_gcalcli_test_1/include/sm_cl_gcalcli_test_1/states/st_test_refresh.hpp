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
using namespace cl_gcalcli;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StTestRefresh : smacc2::SmaccState<StTestRefresh, SmClGcalcliTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Timer to proceed to next test
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StTestEventDetect, SUCCESS>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Test CbRefreshAgenda - sync agenda refresh
    configure_orthogonal<OrCalendar, CbRefreshAgenda>();

    // Wait 3 seconds then proceed
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(3);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "StTestRefresh - Testing CbRefreshAgenda");
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "Refreshing agenda...");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "StTestRefresh - onExit()"); }
};

}  // namespace sm_cl_gcalcli_test_1
