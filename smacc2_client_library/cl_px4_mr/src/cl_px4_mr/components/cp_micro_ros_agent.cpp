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
#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <system_error>
#include <thread>

namespace cl_px4_mr
{
namespace
{
// Reaps the group leader and returns true once no process of the group is left.
bool waitForGroupExit(pid_t group, bool & leaderReaped, std::chrono::milliseconds limit)
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (true)
  {
    if (!leaderReaped)
    {
      int status = 0;
      const pid_t result = waitpid(group, &status, WNOHANG);
      leaderReaped = result == group || (result == -1 && errno == ECHILD);
    }
    if (leaderReaped && kill(-group, 0) == -1 && errno == ESRCH) return true;
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// Body of the watchdog process: stops the agent's group when the state machine dies without
// running shutdown(). Runs after fork() in a threaded process, so async-signal-safe calls only.
[[noreturn]] void runWatchdog(const int * lifePipe, pid_t group)
{
  const int lifeFd = lifePipe[0];
  for (const int sig : {SIGINT, SIGTERM, SIGHUP, SIGQUIT}) signal(sig, SIG_IGN);
  prctl(PR_SET_NAME, "cp_agent_watch");
  if (lifeFd > 0) close_range(0, lifeFd - 1, 0);
  close_range(lifeFd + 1, ~0U, 0);

  // The state machine holds the only write end; end of file means that process is gone
  char byte = 0;
  ssize_t count = 0;
  do
  {
    count = read(lifeFd, &byte, 1);
  } while (count != 0 && !(count == -1 && errno != EINTR));

  kill(-group, SIGTERM);
  const timespec step{0, 20000000L};
  for (int i = 0; i < 100 && !(kill(-group, 0) == -1 && errno == ESRCH); ++i)
  {
    nanosleep(&step, nullptr);
  }
  kill(-group, SIGKILL);
  _exit(0);
}
}  // namespace

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
    RCLCPP_ERROR(
      getLogger(), "CpMicroRosAgent: pipe() failed: %s",
      std::system_category().message(errno).c_str());
    return;
  }

  // Held open by this process only; the watchdog reads its other end
  int lifefd[2];
  if (pipe2(lifefd, O_CLOEXEC) == -1)
  {
    RCLCPP_ERROR(
      getLogger(), "CpMicroRosAgent: pipe2() failed: %s",
      std::system_category().message(errno).c_str());
    close(pipefd[0]);
    close(pipefd[1]);
    return;
  }

  pid_t pid = fork();

  if (pid == 0)
  {
    // Child process: its own process group, so shutdown() can signal the shell and all it starts
    setpgid(0, 0);
    dup2(pipefd[1], STDOUT_FILENO);
    dup2(pipefd[1], STDERR_FILENO);
    // The agent keeps none of this process's descriptors (sockets, DDS shared memory, the pipe)
    close_range(3, ~0U, 0);

    execl("/bin/sh", "/bin/sh", "-c", command_.c_str(), nullptr);

    // If execl returns, it failed
    _exit(1);
  }
  else if (pid > 0)
  {
    // Parent process
    close(pipefd[1]);
    setpgid(pid, pid);  // also here, so the group exists whichever process runs first
    agentPid_ = pid;
    pipeFd_ = pipefd[0];

    // A second child that outlives a crash or SIGKILL of this process and then stops the agent
    watchdogPid_ = fork();
    if (watchdogPid_ == 0) runWatchdog(lifefd, pid);
    close(lifefd[0]);
    if (watchdogPid_ > 0)
    {
      lifeFd_ = lifefd[1];
    }
    else
    {
      RCLCPP_WARN(
        getLogger(), "CpMicroRosAgent: no watchdog, fork() failed: %s",
        std::system_category().message(errno).c_str());
      close(lifefd[1]);
    }

    // Set pipe to non-blocking
    int flags = fcntl(pipeFd_, F_GETFL, 0);
    fcntl(pipeFd_, F_SETFL, flags | O_NONBLOCK);

    // Reader thread draining the pipe (prevents buffer fill); joined in shutdown()
    shutdownRequested_ = false;
    readerThread_ = std::thread(
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

    launched_ = true;
    RCLCPP_INFO(getLogger(), "CpMicroRosAgent: launched with PID %d", agentPid_);
  }
  else
  {
    RCLCPP_ERROR(
      getLogger(), "CpMicroRosAgent: fork() failed: %s",
      std::system_category().message(errno).c_str());
    close(pipefd[0]);
    close(pipefd[1]);
    close(lifefd[0]);
    close(lifefd[1]);
  }
}

void CpMicroRosAgent::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // The reader checks the flag every 100 ms, so the join is bounded; it must happen before the
  // pipe is closed and before this object is destroyed.
  shutdownRequested_ = true;
  if (readerThread_.joinable())
  {
    readerThread_.join();
  }

  if (agentPid_ <= 0)
  {
    return;
  }

  RCLCPP_INFO(getLogger(), "CpMicroRosAgent: shutting down agent (PID %d)", agentPid_);

  // SIGTERM to the whole group, SIGKILL to whatever is left after two seconds
  bool leaderReaped = false;
  kill(-agentPid_, SIGTERM);
  if (!waitForGroupExit(agentPid_, leaderReaped, std::chrono::seconds(2)))
  {
    RCLCPP_WARN(getLogger(), "CpMicroRosAgent: agent still running after SIGTERM, sending SIGKILL");
    kill(-agentPid_, SIGKILL);
    if (!waitForGroupExit(agentPid_, leaderReaped, std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(
        getLogger(), "CpMicroRosAgent: process group %d still exists after SIGKILL", agentPid_);
    }
  }

  // The agent is stopped, so the watchdog has nothing left to do
  if (watchdogPid_ > 0)
  {
    kill(watchdogPid_, SIGKILL);
    waitpid(watchdogPid_, nullptr, 0);
    watchdogPid_ = -1;
  }
  if (lifeFd_ >= 0)
  {
    close(lifeFd_);
    lifeFd_ = -1;
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

}  // namespace cl_px4_mr
