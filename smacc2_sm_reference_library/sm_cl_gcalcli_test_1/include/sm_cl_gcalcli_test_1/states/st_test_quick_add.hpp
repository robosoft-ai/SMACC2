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
struct StTestQuickAdd : smacc2::SmaccState<StTestQuickAdd, SmClGcalcliTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Quick add succeeded
    Transition<EvCbSuccess<CbQuickAdd, OrCalendar>, StDone, SUCCESS>,
    // Quick add failed
    Transition<EvCbFailure<CbQuickAdd, OrCalendar>, StDone, ABORT>,
    // Timeout fallback
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StDone, SUCCESS>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Test CbQuickAdd - add a test event
    // Note: This will create an actual calendar event if gcalcli is configured
    configure_orthogonal<OrCalendar, CbQuickAdd>(
      "sm_cl_gcalcli_test_1 Test Event tomorrow 2pm for 30 minutes");

    // Timeout fallback (10 seconds)
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "StTestQuickAdd - Testing CbQuickAdd");
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "Adding test event via gcalcli quick add...");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "StTestQuickAdd - onExit()"); }
};

}  // namespace sm_cl_gcalcli_test_1
