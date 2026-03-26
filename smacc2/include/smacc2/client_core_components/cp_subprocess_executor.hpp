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
#include <smacc2/common.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
namespace client_core_components
{

/**
 * @brief Result of a subprocess execution
 */
struct SubprocessResult
{
  int exit_code;
  std::string stdout_output;
  std::string stderr_output;
  bool timed_out;
  std::chrono::milliseconds execution_time;
};

/**
 * @brief Generic subprocess execution component for running CLI tools
 *
 * This component provides a thread-safe way to execute external commands
 * and capture their output. It can be used by any client that needs to
 * interact with command-line tools.
 *
 * Example usage:
 *   auto result = executor->executeCommand("gcalcli list", 5000);
 *   if (result.exit_code == 0) {
 *     // Process result.stdout_output
 *   }
 */
class CpSubprocessExecutor : public smacc2::ISmaccComponent
{
public:
  CpSubprocessExecutor() : initialized_(false) {}

  virtual ~CpSubprocessExecutor() = default;

  void onInitialize() override
  {
    if (!initialized_)
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << this->getName() << "] CpSubprocessExecutor initialized");
      initialized_ = true;
    }
  }

  /**
   * @brief Execute a command synchronously
   *
   * @param command The command to execute (passed to shell)
   * @param timeout_ms Timeout in milliseconds (0 = no timeout)
   * @return SubprocessResult containing exit code, stdout, stderr
   */
  SubprocessResult executeCommand(const std::string & command, int timeout_ms = 30000)
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);

    SubprocessResult result;
    result.exit_code = -1;
    result.timed_out = false;

    auto start_time = std::chrono::steady_clock::now();

    RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] Executing: " << command);

    // Execute command and capture output
    std::string full_command = command + " 2>&1";
    FILE * pipe = popen(full_command.c_str(), "r");

    if (!pipe)
    {
      result.stderr_output = "Failed to execute command: popen() failed";
      RCLCPP_ERROR_STREAM(getLogger(), "[" << this->getName() << "] " << result.stderr_output);
      onCommandFailed_(result.stderr_output);
      return result;
    }

    // Read output
    std::array<char, 4096> buffer;
    std::string output;

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
      output += buffer.data();

      // Check timeout
      if (timeout_ms > 0)
      {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > timeout_ms)
        {
          pclose(pipe);
          result.timed_out = true;
          result.stderr_output = "Command timed out after " + std::to_string(timeout_ms) + "ms";
          RCLCPP_WARN_STREAM(getLogger(), "[" << this->getName() << "] " << result.stderr_output);
          onCommandFailed_(result.stderr_output);
          return result;
        }
      }
    }

    int status = pclose(pipe);
    result.exit_code = WEXITSTATUS(status);
    result.stdout_output = output;

    auto end_time = std::chrono::steady_clock::now();
    result.execution_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    RCLCPP_DEBUG_STREAM(
      getLogger(), "[" << this->getName() << "] Command completed with exit code "
                       << result.exit_code << " in " << result.execution_time.count() << "ms");

    if (result.exit_code == 0)
    {
      onCommandCompleted_(result.exit_code, result.stdout_output);
    }
    else
    {
      onCommandFailed_(result.stdout_output);
    }

    return result;
  }

  /**
   * @brief Execute a command asynchronously (fire and forget)
   *
   * The command runs in a separate thread. Results are delivered via signals.
   *
   * @param command The command to execute
   * @param timeout_ms Timeout in milliseconds
   */
  void executeCommandAsync(const std::string & command, int timeout_ms = 30000)
  {
    std::thread([this, command, timeout_ms]() { this->executeCommand(command, timeout_ms); })
      .detach();
  }

  // Signals
  smacc2::SmaccSignal<void(int exit_code, const std::string & output)> onCommandCompleted_;
  smacc2::SmaccSignal<void(const std::string & error)> onCommandFailed_;

  // Signal connection helpers
  template <typename T>
  smacc2::SmaccSignalConnection onCommandCompleted(
    void (T::*callback)(int, const std::string &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onCommandCompleted_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onCommandFailed(
    void (T::*callback)(const std::string &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onCommandFailed_, callback, object);
  }

private:
  bool initialized_;
  std::mutex execution_mutex_;
};

}  // namespace client_core_components
}  // namespace smacc2
