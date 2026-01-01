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

#include <cl_gcalcli/client_behaviors/cb_detect_calendar_event.hpp>

#include <algorithm>

#include <boost/regex.hpp>

namespace cl_gcalcli
{

CbDetectCalendarEvent::CbDetectCalendarEvent(
  const std::string & pattern, bool use_regex, int minutes_before)
: pattern_(pattern),
  use_regex_(use_regex),
  minutes_before_(minutes_before),
  listener_(nullptr),
  client_(nullptr),
  triggered_(false)
{
}

void CbDetectCalendarEvent::onEntry()
{
  RCLCPP_INFO(
    getLogger(),
    "[CbDetectCalendarEvent] Waiting for event matching pattern '%s' (regex=%s, minutes_before=%d)",
    pattern_.c_str(), use_regex_ ? "true" : "false", minutes_before_);

  // Get the client and listener component
  this->requiresClient(client_);
  if (client_)
  {
    listener_ = client_->getEventListener();
  }

  if (!listener_)
  {
    RCLCPP_ERROR(getLogger(), "[CbDetectCalendarEvent] CpCalendarEventListener not available");
    this->postFailureEvent();
    return;
  }

  // Reset triggered events to allow re-detection
  listener_->resetTriggeredEvents();

  // Configure the watch
  EventWatch watch;
  watch.pattern = pattern_;
  watch.use_regex = use_regex_;
  watch.minutes_before = minutes_before_;
  watch.trigger_on_start = true;
  watch.trigger_on_end = false;
  watch.continuous = false;

  listener_->addWatch(watch);

  // Connect to the event started signal
  listener_->onEventStarted(&CbDetectCalendarEvent::onEventStarted, this);
}

void CbDetectCalendarEvent::onExit()
{
  RCLCPP_DEBUG(getLogger(), "[CbDetectCalendarEvent] Exiting");
}

void CbDetectCalendarEvent::onEventStarted(const CalendarEvent & event)
{
  if (triggered_)
  {
    return;  // Already triggered
  }

  // Check if this event matches our pattern
  bool matches = false;
  if (use_regex_)
  {
    try
    {
      boost::regex regex(pattern_, boost::regex::icase);
      matches = boost::regex_search(event.title, regex);
    }
    catch (const boost::regex_error &)
    {
      matches = false;
    }
  }
  else
  {
    std::string title_lower = event.title;
    std::string pattern_lower = pattern_;
    std::transform(title_lower.begin(), title_lower.end(), title_lower.begin(), ::tolower);
    std::transform(pattern_lower.begin(), pattern_lower.end(), pattern_lower.begin(), ::tolower);
    matches = title_lower.find(pattern_lower) != std::string::npos;
  }

  if (matches)
  {
    triggered_ = true;
    detected_event_ = event;

    RCLCPP_INFO(getLogger(), "[CbDetectCalendarEvent] Event detected: %s", event.title.c_str());

    this->postSuccessEvent();
  }
}

}  // namespace cl_gcalcli
