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
#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_core_components/cp_subprocess_executor.hpp>
#include <smacc2/smacc.hpp>

#include <cl_gcalcli/events.hpp>
#include <cl_gcalcli/types.hpp>

namespace cl_gcalcli
{

/**
 * @brief Component that manages gcalcli connection health
 *
 * This component monitors the connection to Google Calendar via gcalcli.
 * It performs periodic heartbeat checks using "gcalcli list" and tracks
 * consecutive failures. After max_consecutive_failures, it emits
 * EvConnectionLost events.
 *
 * The component uses CpSubprocessExecutor from smacc2 core for command execution.
 */
class CpGcalcliConnection : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpGcalcliConnection();
  virtual ~CpGcalcliConnection() = default;

  void onInitialize() override;

  /**
   * @brief Configure the gcalcli connection parameters
   */
  void configure(const GcalcliConfig & config);

  /**
   * @brief Get current connection state
   */
  ConnectionState getConnectionState() const;

  /**
   * @brief Check if connected to Google Calendar
   */
  bool isConnected() const;

  /**
   * @brief Manually trigger a connection check
   * @return true if connection succeeded
   */
  bool checkConnection();

  /**
   * @brief Restart connection after failure
   */
  void restartConnection();

  /**
   * @brief Execute a gcalcli command
   * @param args Command arguments (e.g., "agenda --tsv")
   * @param timeout_ms Command timeout in milliseconds
   * @return SubprocessResult with output and exit code
   */
  smacc2::client_core_components::SubprocessResult executeGcalcli(
    const std::string & args, int timeout_ms = 30000);

  /**
   * @brief Get the gcalcli configuration
   */
  const GcalcliConfig & getConfig() const { return config_; }

  // Signals
  smacc2::SmaccSignal<void()> onConnectionLost_;
  smacc2::SmaccSignal<void()> onConnectionRestored_;
  smacc2::SmaccSignal<void()> onAuthenticationRequired_;

  // Signal connection helpers
  template <typename T>
  smacc2::SmaccSignalConnection onConnectionLost(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onConnectionLost_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onConnectionRestored(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onConnectionRestored_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onAuthenticationRequired(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(
      onAuthenticationRequired_, callback, object);
  }

  // Event posting function - set via onStateOrthogonalAllocation
  std::function<void()> postConnectionLostEvent_;
  std::function<void()> postConnectionRestoredEvent_;
  std::function<void()> postAuthenticationRequiredEvent_;

  /**
   * @brief Template method for type-safe event posting setup
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    postConnectionLostEvent_ = [this]()
    { this->postEvent<EvConnectionLost<CpGcalcliConnection, TOrthogonal>>(); };

    postConnectionRestoredEvent_ = [this]()
    { this->postEvent<EvConnectionRestored<CpGcalcliConnection, TOrthogonal>>(); };

    postAuthenticationRequiredEvent_ = [this]()
    { this->postEvent<EvAuthenticationRequired<CpGcalcliConnection, TOrthogonal>>(); };
  }

protected:
  /**
   * @brief Periodic update for heartbeat (called by SignalDetector)
   */
  void update() override;

private:
  /**
   * @brief Perform the heartbeat check
   */
  void performHeartbeat();

  /**
   * @brief Handle connection state change
   */
  void handleConnectionStateChange(bool success, const std::string & output);

  /**
   * @brief Check if output indicates authentication failure
   */
  bool isAuthenticationError(const std::string & output) const;

  GcalcliConfig config_;
  ConnectionState connection_state_;
  int consecutive_failures_;
  bool initialized_;

  std::chrono::steady_clock::time_point last_heartbeat_time_;

  smacc2::client_core_components::CpSubprocessExecutor * subprocess_executor_;
  mutable std::mutex state_mutex_;
};

}  // namespace cl_gcalcli
