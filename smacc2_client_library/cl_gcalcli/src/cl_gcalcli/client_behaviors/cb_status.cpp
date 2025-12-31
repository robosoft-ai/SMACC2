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

#include <cl_gcalcli/client_behaviors/cb_status.hpp>

namespace cl_gcalcli
{

void CbStatus::onEntry()
{
  this->requiresClient(client_);

  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "[CbStatus] ClGcalcli client not available");
    connection_state_ = ConnectionState::ERROR;
    return;
  }

  auto * connection = client_->getConnection();
  auto * poller = client_->getPoller();

  if (connection)
  {
    connection_state_ = connection->getConnectionState();
  }
  else
  {
    connection_state_ = ConnectionState::DISCONNECTED;
  }

  if (poller)
  {
    events_ = poller->getEvents();
    active_events_ = poller->getActiveEvents();
  }

  RCLCPP_INFO(
    getLogger(), "[CbStatus] Connection: %s, Events: %zu, Active: %zu",
    connection_state_ == ConnectionState::CONNECTED        ? "CONNECTED"
    : connection_state_ == ConnectionState::DISCONNECTED   ? "DISCONNECTED"
    : connection_state_ == ConnectionState::AUTHENTICATING ? "AUTHENTICATING"
                                                           : "ERROR",
    events_.size(), active_events_.size());
}

}  // namespace cl_gcalcli
