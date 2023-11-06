// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_state_machine.hpp>

#include <rclcpp_action/client.hpp>
#include <thread>

namespace smacc2
{
namespace client_bases
{
struct ProcessInfo
{
  pid_t pid;    // PID del proceso hijo
  FILE * pipe;  // Pipe para la salida del proceso hijo
};

ProcessInfo runProcess(const char * command);
void killGrandchildren(pid_t originalPid);
void killProcessesRecursive(pid_t pid);

class ClRosLaunch2 : public ISmaccClient
{
public:
  ClRosLaunch2();

  ClRosLaunch2(std::string packageName, std::string launchFilename);

  virtual ~ClRosLaunch2();

  void launch();

  void stop();

  static std::future<std::string> executeRosLaunch(
    std::string packageName, std::string launchFilename, std::function<bool()> cancelCondition,
    ClRosLaunch2 * client = nullptr);

  // static std::string executeRosLaunch(
  //    std::string packageName, std::string launchFilename, std::function<bool()> cancelCondition);

  std::string packageName_;

  std::string launchFileName_;

  pid_t launchPid_;

protected:
  // std::future<std::string> result_;
  std::future<std::string> result_;

  typedef std::function<void> cancelCallback;

  static std::map<std::future<void>, cancelCallback> detached_futures_;

  std::atomic<bool> cancellationToken_ = ATOMIC_VAR_INIT(false);
};
}  // namespace client_bases
}  // namespace smacc2
