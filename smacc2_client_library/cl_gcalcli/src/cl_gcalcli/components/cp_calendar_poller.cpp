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

#include <cl_gcalcli/components/cp_calendar_poller.hpp>

#include <boost/regex.hpp>
#include <iomanip>
#include <sstream>

namespace cl_gcalcli
{

CpCalendarPoller::CpCalendarPoller() : connection_(nullptr), initialized_(false) {}

void CpCalendarPoller::onInitialize()
{
  if (!initialized_)
  {
    requiresComponent(connection_);

    if (connection_ == nullptr)
    {
      RCLCPP_ERROR(getLogger(), "[CpCalendarPoller] CpGcalcliConnection component not found!");
      return;
    }

    last_poll_attempt_ = std::chrono::steady_clock::now();
    initialized_ = true;

    RCLCPP_INFO(getLogger(), "[CpCalendarPoller] Initialized");
  }
}

bool CpCalendarPoller::refreshAgenda()
{
  if (!connection_ || !connection_->isConnected())
  {
    RCLCPP_WARN(getLogger(), "[CpCalendarPoller] Cannot refresh - not connected");
    return false;
  }

  const auto & config = connection_->getConfig();

  // Build the agenda command with TSV output
  std::stringstream args;
  args << "agenda --tsv --nocolor";
  args << " --nodeclined";

  // Add date range
  auto now = std::chrono::system_clock::now();
  auto end = now + std::chrono::hours(24 * config.agenda_days);

  auto now_t = std::chrono::system_clock::to_time_t(now);
  auto end_t = std::chrono::system_clock::to_time_t(end);

  std::tm now_tm = *std::localtime(&now_t);
  std::tm end_tm = *std::localtime(&end_t);

  args << " \"" << std::put_time(&now_tm, "%m/%d/%Y") << "\"";
  args << " \"" << std::put_time(&end_tm, "%m/%d/%Y") << "\"";

  auto result = connection_->executeGcalcli(args.str(), 30000);

  if (result.exit_code != 0 || result.timed_out)
  {
    RCLCPP_WARN(
      getLogger(), "[CpCalendarPoller] Agenda fetch failed: %s", result.stdout_output.c_str());
    return false;
  }

  // Parse the TSV output
  auto events = parseTsvOutput(result.stdout_output);

  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    cached_events_ = events;
    last_poll_time_ = std::chrono::system_clock::now();
  }

  RCLCPP_DEBUG(getLogger(), "[CpCalendarPoller] Refreshed agenda: %zu events", events.size());

  // Emit signal and post event
  onAgendaUpdated_(events);
  if (postAgendaUpdatedEvent_)
  {
    postAgendaUpdatedEvent_(events);
  }

  return true;
}

std::vector<CalendarEvent> CpCalendarPoller::getEvents() const
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  return cached_events_;
}

std::vector<CalendarEvent> CpCalendarPoller::findEvents(
  const std::string & pattern, bool use_regex) const
{
  std::lock_guard<std::mutex> lock(events_mutex_);

  std::vector<CalendarEvent> matches;

  if (use_regex)
  {
    try
    {
      boost::regex regex(pattern, boost::regex::icase);
      for (const auto & event : cached_events_)
      {
        if (boost::regex_search(event.title, regex))
        {
          matches.push_back(event);
        }
      }
    }
    catch (const boost::regex_error & e)
    {
      RCLCPP_ERROR(getLogger(), "[CpCalendarPoller] Invalid regex pattern: %s", e.what());
    }
  }
  else
  {
    for (const auto & event : cached_events_)
    {
      if (event.title.find(pattern) != std::string::npos)
      {
        matches.push_back(event);
      }
    }
  }

  return matches;
}

std::vector<CalendarEvent> CpCalendarPoller::getEventsInWindow(
  std::chrono::system_clock::time_point start, std::chrono::system_clock::time_point end) const
{
  std::lock_guard<std::mutex> lock(events_mutex_);

  std::vector<CalendarEvent> matches;
  for (const auto & event : cached_events_)
  {
    // Event overlaps with window if it starts before window end and ends after window start
    if (event.start_time < end && event.end_time > start)
    {
      matches.push_back(event);
    }
  }

  return matches;
}

std::vector<CalendarEvent> CpCalendarPoller::getActiveEvents() const
{
  std::lock_guard<std::mutex> lock(events_mutex_);

  std::vector<CalendarEvent> active;
  for (const auto & event : cached_events_)
  {
    if (event.isActiveNow())
    {
      active.push_back(event);
    }
  }

  return active;
}

std::chrono::system_clock::time_point CpCalendarPoller::getLastPollTime() const
{
  std::lock_guard<std::mutex> lock(events_mutex_);
  return last_poll_time_;
}

void CpCalendarPoller::update()
{
  if (!initialized_ || !connection_)
  {
    return;
  }

  if (!connection_->isConnected())
  {
    return;
  }

  const auto & config = connection_->getConfig();
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_poll_attempt_);

  if (elapsed >= config.poll_interval)
  {
    refreshAgenda();
    last_poll_attempt_ = now;
  }
}

std::vector<CalendarEvent> CpCalendarPoller::parseTsvOutput(const std::string & output)
{
  std::vector<CalendarEvent> events;
  std::istringstream stream(output);
  std::string line;

  while (std::getline(stream, line))
  {
    // Skip empty lines
    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos)
    {
      continue;
    }

    auto event = parseTsvLine(line);
    if (event.has_value())
    {
      events.push_back(event.value());
    }
  }

  return events;
}

std::optional<CalendarEvent> CpCalendarPoller::parseTsvLine(const std::string & line)
{
  // TSV columns: Start_Date | Start_Time | End_Date | End_Time | Title | Location | Description | Calendar
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;

  while (std::getline(stream, field, '\t'))
  {
    fields.push_back(field);
  }

  // We need at least 5 fields (date/time + title)
  if (fields.size() < 5)
  {
    RCLCPP_DEBUG(
      getLogger(), "[CpCalendarPoller] Skipping line with insufficient fields: %s", line.c_str());
    return std::nullopt;
  }

  CalendarEvent event;

  // Parse start time
  auto start = parseDateTime(fields[0], fields[1]);
  if (!start.has_value())
  {
    RCLCPP_DEBUG(
      getLogger(), "[CpCalendarPoller] Failed to parse start time: %s %s", fields[0].c_str(),
      fields[1].c_str());
    return std::nullopt;
  }
  event.start_time = start.value();

  // Parse end time
  auto end = parseDateTime(fields[2], fields[3]);
  if (!end.has_value())
  {
    RCLCPP_DEBUG(
      getLogger(), "[CpCalendarPoller] Failed to parse end time: %s %s", fields[2].c_str(),
      fields[3].c_str());
    return std::nullopt;
  }
  event.end_time = end.value();

  // Title (required)
  event.title = fields[4];

  // Optional fields
  if (fields.size() > 5) event.location = fields[5];
  if (fields.size() > 6) event.description = fields[6];
  if (fields.size() > 7) event.calendar_name = fields[7];

  // Check for all-day event (typically has no time or 00:00)
  event.is_all_day = (fields[1].empty() || fields[1] == "00:00" || fields[1] == "12:00am");

  // Generate ID
  event.id = generateEventId(event);

  return event;
}

std::optional<std::chrono::system_clock::time_point> CpCalendarPoller::parseDateTime(
  const std::string & date_str, const std::string & time_str)
{
  std::tm tm = {};

  // Parse date - gcalcli TSV uses YYYY-MM-DD format
  std::istringstream date_stream(date_str);
  char sep1, sep2;
  int year, month, day;

  if (date_stream >> year >> sep1 >> month >> sep2 >> day && sep1 == '-' && sep2 == '-')
  {
    // YYYY-MM-DD format (gcalcli TSV)
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
  }
  else
  {
    // Try MM/DD/YYYY format as fallback
    date_stream.clear();
    date_stream.str(date_str);
    if (date_stream >> month >> sep1 >> day >> sep2 >> year && sep1 == '/' && sep2 == '/')
    {
      tm.tm_mon = month - 1;
      tm.tm_mday = day;
      tm.tm_year = year - 1900;
    }
    else
    {
      RCLCPP_DEBUG(getLogger(), "[CpCalendarPoller] Failed to parse date: %s", date_str.c_str());
      return std::nullopt;
    }
  }

  // Parse time - gcalcli TSV uses 24-hour HH:MM format
  if (!time_str.empty())
  {
    std::istringstream time_stream(time_str);
    char colon;
    int hour, minute;

    if (time_stream >> hour >> colon >> minute && colon == ':')
    {
      tm.tm_hour = hour;
      tm.tm_min = minute;

      // Check for am/pm suffix (for non-TSV formats)
      std::string rest;
      std::getline(time_stream, rest);
      if (!rest.empty())
      {
        // Remove leading spaces
        size_t start = rest.find_first_not_of(" ");
        if (start != std::string::npos)
        {
          rest = rest.substr(start);
        }
        // Check for am/pm
        if (rest == "pm" || rest == "PM" || rest == "p" || rest == "P")
        {
          if (hour != 12) tm.tm_hour += 12;
        }
        else if (rest == "am" || rest == "AM" || rest == "a" || rest == "A")
        {
          if (hour == 12) tm.tm_hour = 0;
        }
      }
    }
    else
    {
      RCLCPP_DEBUG(getLogger(), "[CpCalendarPoller] Failed to parse time: %s", time_str.c_str());
      return std::nullopt;
    }
  }

  tm.tm_sec = 0;
  tm.tm_isdst = -1;  // Let mktime determine DST

  std::time_t time = std::mktime(&tm);
  if (time == -1)
  {
    return std::nullopt;
  }

  return std::chrono::system_clock::from_time_t(time);
}

std::string CpCalendarPoller::generateEventId(const CalendarEvent & event)
{
  // Generate a unique ID based on title and start time
  auto start_t = std::chrono::system_clock::to_time_t(event.start_time);
  std::stringstream ss;
  ss << event.title << "_" << start_t << "_" << event.calendar_name;
  return ss.str();
}

}  // namespace cl_gcalcli
