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

#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace cl_gcalcli
{

/**
 * @brief Connection state for gcalcli
 */
enum class ConnectionState
{
  CONNECTED,
  DISCONNECTED,
  AUTHENTICATING,
  ERROR
};

/**
 * @brief Represents a Google Calendar event
 */
struct CalendarEvent
{
  std::string id;
  std::string title;
  std::string calendar_name;
  std::string location;
  std::string description;
  std::chrono::system_clock::time_point start_time;
  std::chrono::system_clock::time_point end_time;
  bool is_all_day = false;

  /**
   * @brief Check if the event is currently active (ongoing)
   */
  bool isActiveNow() const
  {
    auto now = std::chrono::system_clock::now();
    return now >= start_time && now < end_time;
  }

  /**
   * @brief Check if the event has started within the last N minutes
   */
  bool hasStartedWithinMinutes(int minutes) const
  {
    auto now = std::chrono::system_clock::now();
    auto window_start = start_time - std::chrono::minutes(minutes);
    return now >= window_start && now < end_time;
  }

  /**
   * @brief Check if the event will start within the next N minutes
   */
  bool willStartWithinMinutes(int minutes) const
  {
    auto now = std::chrono::system_clock::now();
    return now < start_time && now >= (start_time - std::chrono::minutes(minutes));
  }

  /**
   * @brief Check if the event has ended
   */
  bool hasEnded() const
  {
    auto now = std::chrono::system_clock::now();
    return now >= end_time;
  }

  /**
   * @brief Get minutes until event starts (negative if already started)
   */
  int minutesUntilStart() const
  {
    auto now = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::minutes>(start_time - now);
    return static_cast<int>(diff.count());
  }
};

/**
 * @brief Configuration for gcalcli client
 */
struct GcalcliConfig
{
  /// Path to gcalcli executable (default: "gcalcli" from PATH)
  std::string gcalcli_path = "gcalcli";

  /// Optional config folder for gcalcli (if not using default)
  std::optional<std::string> config_folder;

  /// Calendars to monitor (empty = all calendars)
  std::vector<std::string> calendars;

  /// How often to poll for agenda updates
  std::chrono::seconds poll_interval{30};

  /// How often to check connection health (heartbeat)
  std::chrono::seconds heartbeat_interval{60};

  /// Number of days ahead to fetch in agenda
  int agenda_days = 7;

  /// Number of consecutive failures before connection is considered lost
  int max_consecutive_failures = 3;
};

/**
 * @brief Event watch configuration for CpCalendarEventListener
 */
struct EventWatch
{
  /// Pattern to match event titles (regex or exact)
  std::string pattern;

  /// True = regex matching, False = exact string matching
  bool use_regex = false;

  /// Trigger N minutes before event starts (0 = at start time)
  int minutes_before = 0;

  /// Post event when event starts
  bool trigger_on_start = true;

  /// Post event when event ends
  bool trigger_on_end = false;

  /// Keep watching (true) or one-shot (false)
  bool continuous = false;
};

}  // namespace cl_gcalcli
