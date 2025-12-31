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
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <cl_gcalcli/components/cp_gcalcli_connection.hpp>
#include <cl_gcalcli/events.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief Component that polls Google Calendar and parses event data
 *
 * This component periodically fetches the agenda from Google Calendar
 * using gcalcli's TSV output format and parses it into CalendarEvent
 * structures.
 *
 * TSV Format:
 * Start_Date | Start_Time | End_Date | End_Time | Title | Location | Description | Calendar
 */
class CpCalendarPoller : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpCalendarPoller();
  virtual ~CpCalendarPoller() = default;

  void onInitialize() override;

  /**
   * @brief Force an immediate agenda refresh
   * @return true if refresh was successful
   */
  bool refreshAgenda();

  /**
   * @brief Get cached list of calendar events
   */
  std::vector<CalendarEvent> getEvents() const;

  /**
   * @brief Get events matching a title pattern
   * @param pattern Pattern to match (regex or exact based on use_regex)
   * @param use_regex If true, use regex matching
   */
  std::vector<CalendarEvent> findEvents(const std::string & pattern, bool use_regex = false) const;

  /**
   * @brief Get events happening within a time window
   */
  std::vector<CalendarEvent> getEventsInWindow(
    std::chrono::system_clock::time_point start, std::chrono::system_clock::time_point end) const;

  /**
   * @brief Get currently active events
   */
  std::vector<CalendarEvent> getActiveEvents() const;

  /**
   * @brief Get the time of the last successful poll
   */
  std::chrono::system_clock::time_point getLastPollTime() const;

  // Signal for agenda updates
  smacc2::SmaccSignal<void(const std::vector<CalendarEvent> &)> onAgendaUpdated_;

  // Signal connection helper
  template <typename T>
  smacc2::SmaccSignalConnection onAgendaUpdated(
    void (T::*callback)(const std::vector<CalendarEvent> &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onAgendaUpdated_, callback, object);
  }

  // Event posting function - set via onStateOrthogonalAllocation
  std::function<void(const std::vector<CalendarEvent> &)> postAgendaUpdatedEvent_;

  /**
   * @brief Template method for type-safe event posting setup
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    postAgendaUpdatedEvent_ = [this](const std::vector<CalendarEvent> & events)
    {
      auto ev = new EvAgendaUpdated<CpCalendarPoller, TOrthogonal>();
      ev->events = events;
      this->postEvent(ev);
    };
  }

protected:
  /**
   * @brief Periodic update for polling (called by SignalDetector)
   */
  void update() override;

private:
  /**
   * @brief Parse TSV output into CalendarEvent structures
   */
  std::vector<CalendarEvent> parseTsvOutput(const std::string & output);

  /**
   * @brief Parse a single TSV line into a CalendarEvent
   */
  std::optional<CalendarEvent> parseTsvLine(const std::string & line);

  /**
   * @brief Parse date and time strings into time_point
   * @param date_str Date string (MM/DD/YYYY or similar)
   * @param time_str Time string (HH:MM or similar)
   */
  std::optional<std::chrono::system_clock::time_point> parseDateTime(
    const std::string & date_str, const std::string & time_str);

  /**
   * @brief Generate a unique ID for an event
   */
  std::string generateEventId(const CalendarEvent & event);

  CpGcalcliConnection * connection_;
  std::vector<CalendarEvent> cached_events_;
  std::chrono::system_clock::time_point last_poll_time_;
  std::chrono::steady_clock::time_point last_poll_attempt_;
  bool initialized_;

  mutable std::mutex events_mutex_;
};

}  // namespace cl_gcalcli
