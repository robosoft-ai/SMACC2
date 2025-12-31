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

#include <string>

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/components/cp_calendar_event_listener.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief Async behavior that waits for a calendar event's start time to arrive
 *
 * This behavior configures a watch pattern on the CpCalendarEventListener
 * and waits for an event matching the pattern to start. When the event
 * start time arrives (minus minutes_before), it posts EvCbSuccess.
 *
 * Usage in state:
 * @code
 * static void staticConfigure() {
 *   // Wait for "Standup" events, trigger 5 minutes before start
 *   configure<OrCalendar, CbDetectCalendarEvent>(".*Standup.*", true, 5);
 * }
 * @endcode
 */
class CbDetectCalendarEvent : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Construct with pattern configuration
   * @param pattern Pattern to match event titles
   * @param use_regex If true, use regex matching; if false, substring match
   * @param minutes_before Trigger N minutes before event starts (0 = at start)
   */
  CbDetectCalendarEvent(const std::string & pattern, bool use_regex = false, int minutes_before = 0);

  virtual ~CbDetectCalendarEvent() = default;

  void onEntry() override;
  void onExit() override;

  /**
   * @brief Get the detected event (if any)
   */
  std::optional<CalendarEvent> getDetectedEvent() const { return detected_event_; }

private:
  /**
   * @brief Callback when an event starts
   */
  void onEventStarted(const CalendarEvent & event);

  std::string pattern_;
  bool use_regex_;
  int minutes_before_;

  CpCalendarEventListener * listener_;
  ClGcalcli * client_;

  std::optional<CalendarEvent> detected_event_;
  bool triggered_;
};

}  // namespace cl_gcalcli
