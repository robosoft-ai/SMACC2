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

#include <cl_gcalcli/client_behaviors/cb_refresh_agenda.hpp>

namespace cl_gcalcli
{

void CbRefreshAgenda::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbRefreshAgenda] Refreshing calendar agenda");

  success_ = false;

  this->requiresClient(client_);

  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "[CbRefreshAgenda] ClGcalcli client not available");
    return;
  }

  auto * poller = client_->getPoller();
  if (!poller)
  {
    RCLCPP_ERROR(getLogger(), "[CbRefreshAgenda] CpCalendarPoller not available");
    return;
  }

  success_ = poller->refreshAgenda();

  if (success_)
  {
    auto events = poller->getEvents();
    RCLCPP_INFO(getLogger(), "[CbRefreshAgenda] Agenda refreshed: %zu events", events.size());
  }
  else
  {
    RCLCPP_WARN(getLogger(), "[CbRefreshAgenda] Agenda refresh failed");
  }
}

}  // namespace cl_gcalcli
