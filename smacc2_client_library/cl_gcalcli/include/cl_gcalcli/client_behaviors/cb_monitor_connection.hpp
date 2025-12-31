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

#include <smacc2/smacc_client_behavior.hpp>
#include <smacc2/smacc_updatable.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/components/cp_gcalcli_connection.hpp>

namespace cl_gcalcli
{

/**
 * @brief Sync behavior that continuously monitors connection health
 *
 * This behavior connects to connection state signals and posts
 * EvConnectionLost events when the connection is lost. Use it
 * alongside other behaviors to handle connection failures.
 */
class CbMonitorConnection : public smacc2::SmaccClientBehavior
{
public:
  CbMonitorConnection() = default;
  virtual ~CbMonitorConnection() = default;

  void onEntry() override;
  void onExit() override;

private:
  void onConnectionLost();
  void onConnectionRestored();

  ClGcalcli * client_;
  CpGcalcliConnection * connection_;
};

}  // namespace cl_gcalcli
