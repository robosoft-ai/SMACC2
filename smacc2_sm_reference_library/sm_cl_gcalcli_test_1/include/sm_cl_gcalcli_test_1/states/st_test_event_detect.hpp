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
struct StTestEventDetect : smacc2::SmaccState<StTestEventDetect, SmClGcalcliTest1>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct EVENT_DETECTED : SUCCESS
  {
  };
  struct CONNECTION_LOST : ABORT
  {
  };

  // TRANSITION TABLE
  typedef mpl::list<
    // Calendar event started - detected matching event
    Transition<
      EvCalendarEventStarted<CpCalendarEventListener, OrCalendar>, StTestQuickAdd, EVENT_DETECTED>,
    // Connection lost - go back to wait
    Transition<EvConnectionLost<CpGcalcliConnection, OrCalendar>, StWaitConnection, CONNECTION_LOST>>
    // Timeout fallback commented out - wait indefinitely for event detection
    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StTestQuickAdd, SUCCESS>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Test CbDetectCalendarEvent - watch for events matching "TestEvent"
    // Using regex=false, minutes_before=5
    configure_orthogonal<OrCalendar, CbDetectCalendarEvent>("TestEvent", false, 0);

    // Test CbMonitorConnection - continuous connection monitoring
    configure_orthogonal<OrCalendar, CbMonitorConnection>();

    // Timeout fallback commented out - wait indefinitely for event detection
    // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(60);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "StTestEventDetect - Testing CbDetectCalendarEvent + CbMonitorConnection");
    RCLCPP_INFO(getLogger(), "====================================");
    RCLCPP_INFO(getLogger(), "Watching for 'TestEvent' calendar events (0 min before)...");
    RCLCPP_INFO(getLogger(), "Monitoring connection health...");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "StTestEventDetect - onExit()"); }
};

}  // namespace sm_cl_gcalcli_test_1
