// Copyright 2025 RobosoftAI Inc.
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
#include <thread>
#include <string>
#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <smacc2/smacc.hpp>

namespace sm_multithread_test_1
{
using namespace cl_ros2_timer;

/**
 * @brief Custom timer behavior that simulates work and logs thread IDs
 *
 * This behavior demonstrates concurrent execution in multi-threaded mode by:
 * 1. Logging the thread ID on each timer tick
 * 2. Simulating work with a configurable sleep duration
 * 3. Logging START/END to visualize overlapping operations
 *
 * In single-threaded mode, you'll see:
 * - Same thread ID for all callbacks
 * - Serial execution (no overlap)
 *
 * In multi-threaded mode, you'll see:
 * - Different thread IDs for concurrent callbacks
 * - Overlapping START/END timestamps
 */
class CbTimerWithWorkSimulation : public smacc2::SmaccClientBehavior
{
public:
  /**
   * @param timer_name Identifier for this timer (e.g., "A", "B", "C", "D")
   * @param work_duration How long to simulate work on each tick
   */
  explicit CbTimerWithWorkSimulation(
    std::string timer_name,
    std::chrono::milliseconds work_duration)
  : timerName_(timer_name),
    workDuration_(work_duration),
    tickCount_(0)
  {
  }

  void onEntry() override
  {
    // Get the timer client
    this->requiresClient(timerClient_);

    // Get the core timer component
    smacc2::client_core_components::CpRos2Timer * timerComponent;
    this->requiresComponent(timerComponent);

    // Connect to the timer tick callback
    timerComponent->onTimerTick(&CbTimerWithWorkSimulation::onTimerCallback, this);

    RCLCPP_INFO(
      getLogger(),
      "[Timer-%s] Behavior initialized - will simulate %ldms work per tick",
      timerName_.c_str(),
      workDuration_.count()
    );
  }

  void onExit() override
  {
    RCLCPP_INFO(
      getLogger(),
      "[Timer-%s] Behavior exiting after %d ticks",
      timerName_.c_str(),
      tickCount_
    );
  }

private:
  std::string timerName_;
  std::chrono::milliseconds workDuration_;
  int tickCount_;
  ClRos2Timer * timerClient_;

  /**
   * @brief Called on each timer tick - simulates work and logs thread info
   */
  void onTimerCallback()
  {
    tickCount_++;

    // Get current thread ID
    auto thread_id = std::this_thread::get_id();
    std::size_t thread_hash = std::hash<std::thread::id>{}(thread_id);

    // Log START with thread ID
    RCLCPP_INFO(
      getLogger(),
      "[Timer-%s] Tick %3d START (Thread: %8zu) - simulating %3ldms work",
      timerName_.c_str(),
      tickCount_,
      thread_hash,
      workDuration_.count()
    );

    // Simulate work (this is where the thread would be doing actual computation)
    std::this_thread::sleep_for(workDuration_);

    // Log END with thread ID
    RCLCPP_INFO(
      getLogger(),
      "[Timer-%s] Tick %3d END   (Thread: %8zu)",
      timerName_.c_str(),
      tickCount_,
      thread_hash
    );
  }
};

}  // namespace sm_multithread_test_1
