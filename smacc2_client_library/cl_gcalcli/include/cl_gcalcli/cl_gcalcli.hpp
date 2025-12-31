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

#include <smacc2/client_core_components/cp_subprocess_executor.hpp>
#include <smacc2/smacc_client.hpp>

#include <cl_gcalcli/components/cp_calendar_event_listener.hpp>
#include <cl_gcalcli/components/cp_calendar_poller.hpp>
#include <cl_gcalcli/components/cp_gcalcli_connection.hpp>
#include <cl_gcalcli/events.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief SMACC2 Client for Google Calendar integration via gcalcli
 *
 * This client provides Google Calendar functionality through the gcalcli
 * command-line tool. It follows the pure orchestrator pattern, creating
 * and configuring components that implement the actual functionality.
 *
 * Components:
 * - CpSubprocessExecutor: Generic subprocess execution (from smacc2 core)
 * - CpGcalcliConnection: Connection health monitoring and command execution
 * - CpCalendarPoller: Agenda polling and event parsing
 * - CpCalendarEventListener: Pattern matching and event triggering
 *
 * Usage example:
 * @code
 * class OrCalendar : public smacc2::Orthogonal<OrCalendar> {
 *   void onInitialize() override {
 *     cl_gcalcli::GcalcliConfig config;
 *     config.poll_interval = std::chrono::seconds{30};
 *     this->createClient<cl_gcalcli::ClGcalcli>(config);
 *   }
 * };
 * @endcode
 */
class ClGcalcli : public smacc2::ISmaccClient
{
public:
  ClGcalcli();
  explicit ClGcalcli(const GcalcliConfig & config);
  virtual ~ClGcalcli() = default;

  /**
   * @brief Set configuration before initialization
   */
  void configure(const GcalcliConfig & config);

  /**
   * @brief Get current configuration
   */
  const GcalcliConfig & getConfig() const { return config_; }

  /**
   * @brief Template method for component initialization with orthogonal context
   */
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Create the subprocess executor (from smacc2 core)
    this->createComponent<
      smacc2::client_core_components::CpSubprocessExecutor, TOrthogonal, ClGcalcli>();

    // Create gcalcli-specific components
    auto * connection = this->createComponent<CpGcalcliConnection, TOrthogonal, ClGcalcli>();
    connection->configure(config_);

    this->createComponent<CpCalendarPoller, TOrthogonal, ClGcalcli>();
    this->createComponent<CpCalendarEventListener, TOrthogonal, ClGcalcli>();

    RCLCPP_INFO(getLogger(), "[ClGcalcli] Components created");
  }

  // Convenience accessors for components
  CpGcalcliConnection * getConnection() { return this->getComponent<CpGcalcliConnection>(); }

  CpCalendarPoller * getPoller() { return this->getComponent<CpCalendarPoller>(); }

  CpCalendarEventListener * getEventListener()
  {
    return this->getComponent<CpCalendarEventListener>();
  }

private:
  GcalcliConfig config_;
};

}  // namespace cl_gcalcli
