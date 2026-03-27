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

#include <cl_px4_mr/components/cp_micro_ros_agent.hpp>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <thread>
#include <vector>

namespace cl_px4_mr
{

CpMicroRosAgent::CpMicroRosAgent(std::string command, std::string nodeName)
: command_(std::move(command)), nodeName_(std::move(nodeName))
{
}

CpMicroRosAgent::~CpMicroRosAgent() { shutdown(); }

void CpMicroRosAgent::launch()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (launched_)
  {
    RCLCPP_INFO(getLogger(), "CpMicroRosAgent: already launched, skipping");
    return;
  }

  RCLCPP_INFO(getLogger(), "CpMicroRosAgent: launching '%s'", command_.c_str());

  int pipefd[2];
  if (pipe(pipefd) == -1)
  {
    RCLCPP_ERROR(getLogger(), "CpMicroRosAgent: pipe() failed: %s", strerror(errno));
    return;
  }

  pid_t pid = fork();

  if (pid == 0)
  {
    // Child process
    close(pipefd[0]);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    close(pipefd[1]);

    execl("/bin/sh", "/bin/sh", "-c", command_.c_str(), nullptr);

    // If execl returns, it failed
    _exit(1);
  }
  else if (pid > 0)
  {
    // Parent process
    close(pipefd[1]);
    agentPid_ = pid;
    pipeFd_ = pipefd[0];

    // Set pipe to non-blocking
    int flags = fcntl(pipeFd_, F_GETFL, 0);
    fcntl(pipeFd_, F_SETFL, flags | O_NONBLOCK);

    // Spawn detached reader thread to drain pipe (prevents buffer fill)
    shutdownRequested_ = false;
    std::thread readerThread(
      [this]()
      {
        char buf[256];
        while (!shutdownRequested_)
        {
          ssize_t n = read(pipeFd_, buf, sizeof(buf) - 1);
          if (n > 0)
          {
            buf[n] = '\0';
            RCLCPP_DEBUG(getLogger(), "[micro_ros_agent] %s", buf);
          }
          else
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
        }
      });
    readerThread.detach();

    launched_ = true;
    RCLCPP_INFO(getLogger(), "CpMicroRosAgent: launched with PID %d", agentPid_);
  }
  else
  {
    RCLCPP_ERROR(getLogger(), "CpMicroRosAgent: fork() failed: %s", strerror(errno));
    close(pipefd[0]);
    close(pipefd[1]);
  }
}

void CpMicroRosAgent::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (agentPid_ <= 0)
  {
    return;
  }

  RCLCPP_INFO(getLogger(), "CpMicroRosAgent: shutting down agent (PID %d)", agentPid_);

  shutdownRequested_ = true;

  // Kill process tree recursively
  killProcessesRecursive(agentPid_);

  // Reap zombie with brief retry loop
  for (int i = 0; i < 10; i++)
  {
    int status;
    pid_t result = waitpid(agentPid_, &status, WNOHANG);
    if (result == agentPid_ || result == -1)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Close pipe
  if (pipeFd_ >= 0)
  {
    close(pipeFd_);
    pipeFd_ = -1;
  }

  agentPid_ = -1;
  launched_ = false;

  RCLCPP_INFO(getLogger(), "CpMicroRosAgent: shutdown complete");
}

bool CpMicroRosAgent::isLaunched() const { return launched_; }

pid_t CpMicroRosAgent::getPid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return agentPid_;
}

std::string CpMicroRosAgent::getNodeName() const { return nodeName_; }

void CpMicroRosAgent::killProcessesRecursive(pid_t pid)
{
  std::string command = "pgrep -P " + std::to_string(pid);
  FILE * pipe = popen(command.c_str(), "r");

  if (!pipe)
  {
    return;
  }

  char buffer[128];
  std::string result;

  while (fgets(buffer, sizeof(buffer), pipe) != NULL)
  {
    result += buffer;
  }

  pclose(pipe);

  std::istringstream iss(result);
  pid_t childPid;
  std::vector<pid_t> childPids;

  while (iss >> childPid)
  {
    childPids.push_back(childPid);
  }

  // Kill child processes before killing the parent process
  for (const pid_t & child : childPids)
  {
    killProcessesRecursive(child);
  }

  // After killing all children, kill the parent process
  int res = kill(pid, SIGTERM);
  if (res == 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("CpMicroRosAgent"), "Killed process %d", pid);
  }
}

}  // namespace cl_px4_mr
