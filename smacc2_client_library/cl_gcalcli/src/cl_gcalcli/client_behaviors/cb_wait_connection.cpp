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

#include <cl_gcalcli/client_behaviors/cb_wait_connection.hpp>

#include <thread>

namespace cl_gcalcli
{

CbWaitConnection::CbWaitConnection(std::chrono::seconds timeout)
: timeout_(timeout), client_(nullptr)
{
}

void CbWaitConnection::onEntry()
{
  RCLCPP_INFO(
    getLogger(), "[CbWaitConnection] Waiting for gcalcli connection (timeout: %lds)",
    timeout_.count());

  this->requiresClient(client_);

  if (!client_)
  {
    RCLCPP_ERROR(getLogger(), "[CbWaitConnection] ClGcalcli client not available");
    this->postFailureEvent();
    return;
  }

  auto * connection = client_->getConnection();
  if (!connection)
  {
    RCLCPP_ERROR(getLogger(), "[CbWaitConnection] CpGcalcliConnection not available");
    this->postFailureEvent();
    return;
  }

  auto start = std::chrono::steady_clock::now();
  auto deadline = start + timeout_;

  // Poll for connection with increasing backoff
  std::chrono::milliseconds poll_interval{500};
  const std::chrono::milliseconds max_interval{5000};

  while (std::chrono::steady_clock::now() < deadline)
  {
    // Check if we should force finish
    if (this->isShutdownRequested())
    {
      RCLCPP_INFO(getLogger(), "[CbWaitConnection] Shutdown requested");
      return;
    }

    if (connection->checkConnection())
    {
      RCLCPP_INFO(getLogger(), "[CbWaitConnection] Connection established");
      this->postSuccessEvent();
      return;
    }

    // Wait before next attempt
    std::this_thread::sleep_for(poll_interval);

    // Increase interval with backoff
    poll_interval = std::min(poll_interval * 2, max_interval);
  }

  RCLCPP_ERROR(getLogger(), "[CbWaitConnection] Connection timeout after %lds", timeout_.count());
  this->postFailureEvent();
}

}  // namespace cl_gcalcli
