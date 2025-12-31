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

#include <chrono>

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>

namespace cl_gcalcli
{

/**
 * @brief Async behavior that waits for gcalcli connection
 *
 * This behavior attempts to establish a connection to Google Calendar
 * via gcalcli and posts success when connected or failure on timeout.
 */
class CbWaitConnection : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Construct with optional timeout
   * @param timeout Maximum time to wait for connection
   */
  explicit CbWaitConnection(std::chrono::seconds timeout = std::chrono::seconds{30});

  virtual ~CbWaitConnection() = default;

  void onEntry() override;

private:
  std::chrono::seconds timeout_;
  ClGcalcli * client_;
};

}  // namespace cl_gcalcli
