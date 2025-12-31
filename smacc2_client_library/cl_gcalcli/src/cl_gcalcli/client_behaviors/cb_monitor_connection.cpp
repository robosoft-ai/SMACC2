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

#include <cl_gcalcli/client_behaviors/cb_monitor_connection.hpp>

namespace cl_gcalcli
{

void CbMonitorConnection::onEntry()
{
  RCLCPP_INFO(getLogger(), "[CbMonitorConnection] Starting connection monitoring");

  this->requiresClient(client_);

  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "[CbMonitorConnection] ClGcalcli client not available");
    return;
  }

  connection_ = client_->getConnection();
  if (!connection_)
  {
    RCLCPP_ERROR(getLogger(), "[CbMonitorConnection] CpGcalcliConnection not available");
    return;
  }

  // Connect to connection state signals
  connection_->onConnectionLost(&CbMonitorConnection::onConnectionLost, this);
  connection_->onConnectionRestored(&CbMonitorConnection::onConnectionRestored, this);
}

void CbMonitorConnection::onExit()
{
  RCLCPP_DEBUG(getLogger(), "[CbMonitorConnection] Stopping connection monitoring");
}

void CbMonitorConnection::onConnectionLost()
{
  RCLCPP_WARN(getLogger(), "[CbMonitorConnection] Connection lost detected");
}

void CbMonitorConnection::onConnectionRestored()
{
  RCLCPP_INFO(getLogger(), "[CbMonitorConnection] Connection restored");
}

}  // namespace cl_gcalcli
