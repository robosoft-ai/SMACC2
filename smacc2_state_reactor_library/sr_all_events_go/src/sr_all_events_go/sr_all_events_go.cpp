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

#include <smacc2/common.hpp>
#include <sr_all_events_go/sr_all_events_go.hpp>

namespace smacc2
{
namespace state_reactors
{
using namespace smacc2::introspection;
void SrAllEventsGo::onInitialized()
{
  for (auto type : eventTypes)
  {
    triggeredEvents[type] = false;
  }
}

void SrAllEventsGo::onEventNotified(const std::type_info * eventType)
{
  RCLCPP_INFO_STREAM(
    getLogger(), "[SB ALLEventsGo] RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
  triggeredEvents[eventType] = true;

  for (auto & entry : triggeredEvents)
  {
    RCLCPP_INFO_STREAM(getLogger(), demangleSymbol(entry.first->name()) << " = " << entry.second);
  }
}

bool SrAllEventsGo::triggers()
{
  RCLCPP_INFO_STREAM(getLogger(), "SB All TRIGGERS?");
  for (auto & entry : triggeredEvents)
  {
    if (!entry.second) return false;
  }
  RCLCPP_INFO_STREAM(getLogger(), "SB ALL TRIGGERED");
  return true;
}
}  // namespace state_reactors
}  // namespace smacc2
