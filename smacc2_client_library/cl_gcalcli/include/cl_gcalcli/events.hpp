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

#include <boost/statechart/event.hpp>
#include <string>
#include <vector>

#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{
namespace sc = boost::statechart;

/**
 * @brief Event posted when gcalcli connection is lost
 *
 * Posted after consecutive failures exceed max_consecutive_failures.
 */
template <typename TSource, typename TOrthogonal>
struct EvConnectionLost : sc::event<EvConnectionLost<TSource, TOrthogonal>>
{
};

/**
 * @brief Event posted when gcalcli connection is restored
 *
 * Posted when connection succeeds after being in a lost state.
 */
template <typename TSource, typename TOrthogonal>
struct EvConnectionRestored : sc::event<EvConnectionRestored<TSource, TOrthogonal>>
{
};

/**
 * @brief Event posted when gcalcli authentication is required
 *
 * Posted when gcalcli returns an authentication error.
 */
template <typename TSource, typename TOrthogonal>
struct EvAuthenticationRequired : sc::event<EvAuthenticationRequired<TSource, TOrthogonal>>
{
};

/**
 * @brief Event posted when a calendar event matches a watched pattern
 *
 * This is triggered when an event matching the configured pattern is
 * approaching (based on minutes_before setting).
 */
template <typename TSource, typename TOrthogonal>
struct EvCalendarEventDetected : sc::event<EvCalendarEventDetected<TSource, TOrthogonal>>
{
  CalendarEvent event;
  std::string matched_pattern;
};

/**
 * @brief Event posted when a calendar event starts
 *
 * This is triggered when the current time reaches the event start time
 * (or minutes_before if configured).
 */
template <typename TSource, typename TOrthogonal>
struct EvCalendarEventStarted : sc::event<EvCalendarEventStarted<TSource, TOrthogonal>>
{
  CalendarEvent event;
};

/**
 * @brief Event posted when a calendar event ends
 *
 * This is triggered when the current time reaches the event end time.
 */
template <typename TSource, typename TOrthogonal>
struct EvCalendarEventEnded : sc::event<EvCalendarEventEnded<TSource, TOrthogonal>>
{
  CalendarEvent event;
};

/**
 * @brief Event posted when the agenda is updated
 *
 * Contains the full list of upcoming calendar events from the last poll.
 */
template <typename TSource, typename TOrthogonal>
struct EvAgendaUpdated : sc::event<EvAgendaUpdated<TSource, TOrthogonal>>
{
  std::vector<CalendarEvent> events;
};

}  // namespace cl_gcalcli
