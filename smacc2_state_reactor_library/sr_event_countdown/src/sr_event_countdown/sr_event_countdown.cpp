// Copyright 2021 RobosoftAI Inc.
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

#include <memory>
#include <smacc2/common.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>

namespace smacc2
{
namespace state_reactors
{
using namespace smacc2::introspection;
SrEventCountdown::SrEventCountdown(int eventCount) : eventCount_(eventCount) {}

void SrEventCountdown::onInitialized()
{
  for (auto type : eventTypes)
  {
    triggeredEvents[type] = false;
  }
}

void SrEventCountdown::onEventNotified(const std::type_info * eventType)
{
  eventCount_--;
  RCLCPP_INFO_STREAM(
    getLogger(), "SB COUNTDOWN (" << eventCount_ << ") RECEIVED EVENT OF TYPE:"
                                  << demangleSymbol(eventType->name()));

  // RCLCPP_INFO_STREAM(getLogger(),"SB ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
  // triggeredEvents[eventType] = true;

  // for (auto &entry : triggeredEvents)
  // {
  //     RCLCPP_INFO_STREAM(getLogger(),demangleSymbol(entry.first->name()) << " = " << entry.second);
  // }
}

bool SrEventCountdown::triggers()
{
  if (eventCount_ == 0)
  {
    RCLCPP_INFO_STREAM(getLogger(), "SB COUNTDOWN (" << eventCount_ << ") TRIGGERS!");
    return true;
  }
  else
  {
    return false;
  }

  // RCLCPP_INFO(getLogger(),"SB All TRIGGERS?");
  // for (auto &entry : triggeredEvents)
  // {
  //     if (!entry.second)
  //         return false;
  // }
  // RCLCPP_INFO(getLogger(),"SB ALL TRIGGERED");
  // return true;
}

}  // namespace state_reactors
}  // namespace smacc2
