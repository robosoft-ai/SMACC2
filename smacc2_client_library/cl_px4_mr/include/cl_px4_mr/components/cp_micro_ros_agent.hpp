// Copyright 2025 Robosoft Inc.
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

#include <sys/types.h>
#include <atomic>
#include <mutex>
#include <smacc2/smacc.hpp>
#include <string>

namespace cl_px4_mr
{

class CpMicroRosAgent : public smacc2::ISmaccComponent
{
public:
  CpMicroRosAgent(
    std::string command =
      "ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 2>&1 | tee /tmp/xrce_agent.log",
    std::string nodeName = "/px4_micro_xrce_dds");
  virtual ~CpMicroRosAgent();

  void launch();
  void shutdown();
  bool isLaunched() const;
  pid_t getPid() const;
  std::string getNodeName() const;

private:
  static void killProcessesRecursive(pid_t pid);

  pid_t agentPid_ = -1;
  std::atomic<bool> launched_{false};
  std::atomic<bool> shutdownRequested_{false};
  std::string command_;
  std::string nodeName_;
  mutable std::mutex mutex_;
  int pipeFd_ = -1;
};

}  // namespace cl_px4_mr
