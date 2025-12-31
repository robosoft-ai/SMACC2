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
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <cl_gcalcli/components/cp_calendar_poller.hpp>
#include <cl_gcalcli/events.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief Component that listens for calendar events and triggers notifications
 *
 * This component subscribes to the CpCalendarPoller and monitors events
 * against configured watch patterns. It triggers events when:
 * - An event matching a pattern is detected (EvCalendarEventDetected)
 * - A watched event starts (EvCalendarEventStarted)
 * - A watched event ends (EvCalendarEventEnded)
 *
 * The component tracks triggered events to prevent duplicate notifications.
 */
class CpCalendarEventListener : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpCalendarEventListener();
  virtual ~CpCalendarEventListener() = default;

  void onInitialize() override;

  /**
   * @brief Add a watch pattern for event detection
   */
  void addWatch(const EventWatch & watch);

  /**
   * @brief Remove all watch patterns
   */
  void clearWatches();

  /**
   * @brief Get all configured watch patterns
   */
  std::vector<EventWatch> getWatches() const;

  /**
   * @brief Reset triggered event tracking (allows re-triggering)
   */
  void resetTriggeredEvents();

  // Signals
  smacc2::SmaccSignal<void(const CalendarEvent &, const std::string &)> onEventDetected_;
  smacc2::SmaccSignal<void(const CalendarEvent &)> onEventStarted_;
  smacc2::SmaccSignal<void(const CalendarEvent &)> onEventEnded_;

  // Signal connection helpers
  template <typename T>
  smacc2::SmaccSignalConnection onEventDetected(
    void (T::*callback)(const CalendarEvent &, const std::string &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onEventDetected_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onEventStarted(
    void (T::*callback)(const CalendarEvent &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onEventStarted_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onEventEnded(void (T::*callback)(const CalendarEvent &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onEventEnded_, callback, object);
  }

  // Event posting functions - set via onStateOrthogonalAllocation
  std::function<void(const CalendarEvent &, const std::string &)> postEventDetectedEvent_;
  std::function<void(const CalendarEvent &)> postEventStartedEvent_;
  std::function<void(const CalendarEvent &)> postEventEndedEvent_;

  /**
   * @brief Template method for type-safe event posting setup
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    postEventDetectedEvent_ = [this](const CalendarEvent & event, const std::string & pattern)
    {
      auto ev = new EvCalendarEventDetected<CpCalendarEventListener, TOrthogonal>();
      ev->event = event;
      ev->matched_pattern = pattern;
      this->postEvent(ev);
    };

    postEventStartedEvent_ = [this](const CalendarEvent & event)
    {
      auto ev = new EvCalendarEventStarted<CpCalendarEventListener, TOrthogonal>();
      ev->event = event;
      this->postEvent(ev);
    };

    postEventEndedEvent_ = [this](const CalendarEvent & event)
    {
      auto ev = new EvCalendarEventEnded<CpCalendarEventListener, TOrthogonal>();
      ev->event = event;
      this->postEvent(ev);
    };
  }

protected:
  /**
   * @brief Periodic update for event time checking (called by SignalDetector)
   */
  void update() override;

private:
  /**
   * @brief Handle agenda update from poller
   */
  void onAgendaUpdated(const std::vector<CalendarEvent> & events);

  /**
   * @brief Check if an event matches a watch pattern
   */
  bool matchesPattern(const CalendarEvent & event, const EventWatch & watch) const;

  /**
   * @brief Check if event start should trigger
   */
  bool shouldTriggerStart(const CalendarEvent & event, const EventWatch & watch) const;

  /**
   * @brief Check if event end should trigger
   */
  bool shouldTriggerEnd(const CalendarEvent & event, const EventWatch & watch) const;

  /**
   * @brief Generate a key for tracking triggered events
   */
  std::string getTriggeredKey(const CalendarEvent & event, const std::string & trigger_type) const;

  CpCalendarPoller * poller_;
  std::vector<EventWatch> watches_;
  std::set<std::string> triggered_events_;  // Tracks which events have been triggered
  std::vector<CalendarEvent> current_events_;
  bool initialized_;

  mutable std::mutex mutex_;
};

}  // namespace cl_gcalcli
