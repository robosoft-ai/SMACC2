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

#include <vector>

#include <smacc2/smacc_client_behavior.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief Sync behavior that retrieves current calendar status
 *
 * This behavior provides a snapshot of the current connection state
 * and calendar events. Use it to check system status on state entry.
 */
class CbStatus : public smacc2::SmaccClientBehavior
{
public:
  CbStatus() = default;
  virtual ~CbStatus() = default;

  void onEntry() override;

  /**
   * @brief Get the connection state captured on entry
   */
  ConnectionState getConnectionState() const { return connection_state_; }

  /**
   * @brief Check if connected
   */
  bool isConnected() const { return connection_state_ == ConnectionState::CONNECTED; }

  /**
   * @brief Get events captured on entry
   */
  std::vector<CalendarEvent> getEvents() const { return events_; }

  /**
   * @brief Get currently active events
   */
  std::vector<CalendarEvent> getActiveEvents() const { return active_events_; }

private:
  ClGcalcli * client_;
  ConnectionState connection_state_;
  std::vector<CalendarEvent> events_;
  std::vector<CalendarEvent> active_events_;
};

}  // namespace cl_gcalcli
