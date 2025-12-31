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

#include <cl_gcalcli/components/cp_gcalcli_connection.hpp>

namespace cl_gcalcli
{

CpGcalcliConnection::CpGcalcliConnection()
: connection_state_(ConnectionState::DISCONNECTED),
  consecutive_failures_(0),
  initialized_(false),
  subprocess_executor_(nullptr)
{
}

void CpGcalcliConnection::onInitialize()
{
  if (!initialized_)
  {
    // Get the subprocess executor component
    requiresComponent(subprocess_executor_);

    if (subprocess_executor_ == nullptr)
    {
      RCLCPP_ERROR(getLogger(), "[CpGcalcliConnection] CpSubprocessExecutor component not found!");
      return;
    }

    last_heartbeat_time_ = std::chrono::steady_clock::now();
    initialized_ = true;

    RCLCPP_INFO(
      getLogger(), "[CpGcalcliConnection] Initialized with gcalcli path: %s",
      config_.gcalcli_path.c_str());
  }
}

void CpGcalcliConnection::configure(const GcalcliConfig & config)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  config_ = config;
  RCLCPP_DEBUG(getLogger(), "[CpGcalcliConnection] Configuration updated");
}

ConnectionState CpGcalcliConnection::getConnectionState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_state_;
}

bool CpGcalcliConnection::isConnected() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return connection_state_ == ConnectionState::CONNECTED;
}

bool CpGcalcliConnection::checkConnection()
{
  if (!subprocess_executor_)
  {
    RCLCPP_ERROR(getLogger(), "[CpGcalcliConnection] Subprocess executor not available");
    return false;
  }

  // Use "gcalcli list" as a lightweight connection test
  auto result = executeGcalcli("list", 10000);

  bool success = (result.exit_code == 0 && !result.timed_out);
  handleConnectionStateChange(success, result.stdout_output);

  return success;
}

void CpGcalcliConnection::restartConnection()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  RCLCPP_INFO(getLogger(), "[CpGcalcliConnection] Restarting connection...");

  consecutive_failures_ = 0;
  connection_state_ = ConnectionState::DISCONNECTED;
}

smacc2::client_core_components::SubprocessResult CpGcalcliConnection::executeGcalcli(
  const std::string & args, int timeout_ms)
{
  if (!subprocess_executor_)
  {
    smacc2::client_core_components::SubprocessResult error_result;
    error_result.exit_code = -1;
    error_result.stderr_output = "Subprocess executor not available";
    error_result.timed_out = false;
    return error_result;
  }

  // Build the full command
  std::string command = config_.gcalcli_path;

  // Add config folder if specified
  if (config_.config_folder.has_value())
  {
    command += " --config-folder \"" + config_.config_folder.value() + "\"";
  }

  // Add calendars if specified
  for (const auto & calendar : config_.calendars)
  {
    command += " --calendar \"" + calendar + "\"";
  }

  // Add the command arguments
  command += " " + args;

  RCLCPP_DEBUG(getLogger(), "[CpGcalcliConnection] Executing: %s", command.c_str());

  return subprocess_executor_->executeCommand(command, timeout_ms);
}

void CpGcalcliConnection::update()
{
  if (!initialized_ || !subprocess_executor_)
  {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat_time_);

  if (elapsed >= config_.heartbeat_interval)
  {
    performHeartbeat();
    last_heartbeat_time_ = now;
  }
}

void CpGcalcliConnection::performHeartbeat()
{
  RCLCPP_DEBUG(getLogger(), "[CpGcalcliConnection] Performing heartbeat check...");

  auto result = executeGcalcli("list", 10000);

  bool success = (result.exit_code == 0 && !result.timed_out);
  handleConnectionStateChange(success, result.stdout_output);
}

void CpGcalcliConnection::handleConnectionStateChange(bool success, const std::string & output)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  ConnectionState previous_state = connection_state_;

  if (success)
  {
    consecutive_failures_ = 0;

    if (connection_state_ != ConnectionState::CONNECTED)
    {
      connection_state_ = ConnectionState::CONNECTED;
      RCLCPP_INFO(getLogger(), "[CpGcalcliConnection] Connection established");

      if (
        previous_state == ConnectionState::ERROR || previous_state == ConnectionState::DISCONNECTED)
      {
        onConnectionRestored_();
        if (postConnectionRestoredEvent_)
        {
          postConnectionRestoredEvent_();
        }
      }
    }
  }
  else
  {
    consecutive_failures_++;

    // Check if this is an authentication error
    if (isAuthenticationError(output))
    {
      connection_state_ = ConnectionState::AUTHENTICATING;
      RCLCPP_WARN(getLogger(), "[CpGcalcliConnection] Authentication required");

      onAuthenticationRequired_();
      if (postAuthenticationRequiredEvent_)
      {
        postAuthenticationRequiredEvent_();
      }
    }
    else if (consecutive_failures_ >= config_.max_consecutive_failures)
    {
      if (connection_state_ != ConnectionState::ERROR)
      {
        connection_state_ = ConnectionState::ERROR;
        RCLCPP_ERROR(
          getLogger(), "[CpGcalcliConnection] Connection lost after %d consecutive failures",
          consecutive_failures_);

        onConnectionLost_();
        if (postConnectionLostEvent_)
        {
          postConnectionLostEvent_();
        }
      }
    }
    else
    {
      RCLCPP_WARN(
        getLogger(), "[CpGcalcliConnection] Connection check failed (%d/%d)", consecutive_failures_,
        config_.max_consecutive_failures);
    }
  }
}

bool CpGcalcliConnection::isAuthenticationError(const std::string & output) const
{
  // Check for common gcalcli authentication error patterns
  return output.find("Authentication") != std::string::npos ||
         output.find("authorization") != std::string::npos ||
         output.find("oauth") != std::string::npos ||
         output.find("credentials") != std::string::npos ||
         output.find("token") != std::string::npos;
}

}  // namespace cl_gcalcli
