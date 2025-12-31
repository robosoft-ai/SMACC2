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

#include <cl_gcalcli/components/cp_calendar_event_listener.hpp>

#include <algorithm>

#include <boost/regex.hpp>

namespace cl_gcalcli
{

CpCalendarEventListener::CpCalendarEventListener() : poller_(nullptr), initialized_(false) {}

void CpCalendarEventListener::onInitialize()
{
  if (!initialized_)
  {
    requiresComponent(poller_);

    if (poller_ == nullptr)
    {
      RCLCPP_ERROR(getLogger(), "[CpCalendarEventListener] CpCalendarPoller component not found!");
      return;
    }

    // Subscribe to agenda updates
    poller_->onAgendaUpdated(&CpCalendarEventListener::onAgendaUpdated, this);

    initialized_ = true;
    RCLCPP_INFO(getLogger(), "[CpCalendarEventListener] Initialized");
  }
}

void CpCalendarEventListener::addWatch(const EventWatch & watch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  watches_.push_back(watch);
  RCLCPP_INFO(
    getLogger(), "[CpCalendarEventListener] Added watch pattern: %s (regex=%s)",
    watch.pattern.c_str(), watch.use_regex ? "true" : "false");
}

void CpCalendarEventListener::clearWatches()
{
  std::lock_guard<std::mutex> lock(mutex_);
  watches_.clear();
  RCLCPP_INFO(getLogger(), "[CpCalendarEventListener] Cleared all watch patterns");
}

std::vector<EventWatch> CpCalendarEventListener::getWatches() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return watches_;
}

void CpCalendarEventListener::resetTriggeredEvents()
{
  std::lock_guard<std::mutex> lock(mutex_);
  triggered_events_.clear();
  RCLCPP_INFO(getLogger(), "[CpCalendarEventListener] Reset triggered events tracking");
}

void CpCalendarEventListener::update()
{
  if (!initialized_ || !poller_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  // Check each watch pattern against current events
  for (const auto & watch : watches_)
  {
    for (const auto & event : current_events_)
    {
      if (!matchesPattern(event, watch))
      {
        continue;
      }

      // Check for start trigger
      if (watch.trigger_on_start && shouldTriggerStart(event, watch))
      {
        std::string key = getTriggeredKey(event, "start");

        // Check if already triggered (unless continuous mode)
        if (watch.continuous || triggered_events_.find(key) == triggered_events_.end())
        {
          triggered_events_.insert(key);

          RCLCPP_INFO(
            getLogger(), "[CpCalendarEventListener] Event start triggered: %s",
            event.title.c_str());

          // Emit signals
          onEventStarted_(event);
          if (postEventStartedEvent_)
          {
            postEventStartedEvent_(event);
          }
        }
      }

      // Check for end trigger
      if (watch.trigger_on_end && shouldTriggerEnd(event, watch))
      {
        std::string key = getTriggeredKey(event, "end");

        if (watch.continuous || triggered_events_.find(key) == triggered_events_.end())
        {
          triggered_events_.insert(key);

          RCLCPP_INFO(
            getLogger(), "[CpCalendarEventListener] Event end triggered: %s", event.title.c_str());

          onEventEnded_(event);
          if (postEventEndedEvent_)
          {
            postEventEndedEvent_(event);
          }
        }
      }
    }
  }
}

void CpCalendarEventListener::onAgendaUpdated(const std::vector<CalendarEvent> & events)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_events_ = events;

  // Check for newly detected events matching patterns
  for (const auto & watch : watches_)
  {
    for (const auto & event : events)
    {
      if (matchesPattern(event, watch))
      {
        std::string key = getTriggeredKey(event, "detected");

        if (watch.continuous || triggered_events_.find(key) == triggered_events_.end())
        {
          triggered_events_.insert(key);

          RCLCPP_DEBUG(
            getLogger(), "[CpCalendarEventListener] Event detected matching pattern '%s': %s",
            watch.pattern.c_str(), event.title.c_str());

          onEventDetected_(event, watch.pattern);
          if (postEventDetectedEvent_)
          {
            postEventDetectedEvent_(event, watch.pattern);
          }
        }
      }
    }
  }
}

bool CpCalendarEventListener::matchesPattern(
  const CalendarEvent & event, const EventWatch & watch) const
{
  if (watch.use_regex)
  {
    try
    {
      boost::regex regex(watch.pattern, boost::regex::icase);
      return boost::regex_search(event.title, regex);
    }
    catch (const boost::regex_error & e)
    {
      RCLCPP_ERROR(
        getLogger(), "[CpCalendarEventListener] Invalid regex pattern '%s': %s",
        watch.pattern.c_str(), e.what());
      return false;
    }
  }
  else
  {
    // Case-insensitive exact substring match
    std::string title_lower = event.title;
    std::string pattern_lower = watch.pattern;
    std::transform(title_lower.begin(), title_lower.end(), title_lower.begin(), ::tolower);
    std::transform(pattern_lower.begin(), pattern_lower.end(), pattern_lower.begin(), ::tolower);
    return title_lower.find(pattern_lower) != std::string::npos;
  }
}

bool CpCalendarEventListener::shouldTriggerStart(
  const CalendarEvent & event, const EventWatch & watch) const
{
  auto now = std::chrono::system_clock::now();

  // Calculate the trigger time (start_time - minutes_before)
  auto trigger_time = event.start_time - std::chrono::minutes(watch.minutes_before);

  // We trigger if:
  // 1. Current time is at or past the trigger time
  // 2. Event hasn't ended yet
  // 3. We're within a reasonable window (not more than minutes_before + 1 minute past start)
  auto max_trigger_window = event.start_time + std::chrono::minutes(1);

  return now >= trigger_time && now < event.end_time && now <= max_trigger_window;
}

bool CpCalendarEventListener::shouldTriggerEnd(
  const CalendarEvent & event, const EventWatch & /* watch */) const
{
  auto now = std::chrono::system_clock::now();

  // Trigger end if:
  // 1. Current time is at or past the end time
  // 2. We're within a reasonable window (1 minute past end)
  auto max_trigger_window = event.end_time + std::chrono::minutes(1);

  return now >= event.end_time && now <= max_trigger_window;
}

std::string CpCalendarEventListener::getTriggeredKey(
  const CalendarEvent & event, const std::string & trigger_type) const
{
  // Create a unique key combining event ID and trigger type
  return event.id + "_" + trigger_type;
}

}  // namespace cl_gcalcli
