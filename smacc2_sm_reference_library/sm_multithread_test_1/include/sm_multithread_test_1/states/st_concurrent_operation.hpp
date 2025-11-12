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

#include <smacc2/smacc.hpp>
#include <sm_multithread_test_1/client_behaviors/cb_timer_with_work_simulation.hpp>

namespace sm_multithread_test_1
{
using namespace cl_ros2_timer;
using namespace smacc2::default_transition_tags;
using namespace std::chrono_literals;

// Forward declaration
struct StComplete;

/**
 * @brief Main demonstration state - runs 4 concurrent timers
 *
 * This state configures four orthogonals with timers at different rates,
 * each simulating work of different durations. The goal is to demonstrate
 * concurrent execution in multi-threaded mode.
 *
 * Timer Configuration:
 * - Timer A: 100ms period, 50ms work  (fast, light)
 * - Timer B: 250ms period, 100ms work (medium, medium)
 * - Timer C: 500ms period, 150ms work (slow, heavy)
 * - Timer D: 1000ms period, 200ms work (very slow, very heavy)
 *
 * The state exits after 120 seconds (120 ticks of Timer D).
 */
struct StConcurrentOperation : smacc2::SmaccState<StConcurrentOperation, SmMultithreadTest1>
{
  using SmaccState::SmaccState;

  // Transition after 120 seconds (120 ticks of the 1-second timer)
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimerD>, StComplete, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Configure Timer A: Fast (100ms period), light work (50ms)
    configure_orthogonal<OrTimerA, CbTimerWithWorkSimulation>("A", 50ms);

    // Configure Timer B: Medium (250ms period), medium work (100ms)
    configure_orthogonal<OrTimerB, CbTimerWithWorkSimulation>("B", 100ms);

    // Configure Timer C: Slow (500ms period), heavy work (150ms)
    configure_orthogonal<OrTimerC, CbTimerWithWorkSimulation>("C", 150ms);

    // Configure Timer D: Very slow (1000ms period), very heavy work (200ms)
    configure_orthogonal<OrTimerD, CbTimerWithWorkSimulation>("D", 200ms);

    // Also configure Timer D with countdown to trigger state exit after 120 seconds
    configure_orthogonal<OrTimerD, CbTimerCountdownOnce>(120);
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StConcurrentOperation::runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "╔════════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(getLogger(), "║      CONCURRENT OPERATION STATE - Multi-threaded Demo         ║");
    RCLCPP_INFO(getLogger(), "╚════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "Running 4 concurrent timers for 15 seconds...");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "  Timer A: 100ms period,  50ms work (fast, light)");
    RCLCPP_INFO(getLogger(), "  Timer B: 250ms period, 100ms work (medium, medium)");
    RCLCPP_INFO(getLogger(), "  Timer C: 500ms period, 150ms work (slow, heavy)");
    RCLCPP_INFO(getLogger(), "  Timer D: 1000ms period, 200ms work (very slow, very heavy)");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "👀 Watch for:");
    RCLCPP_INFO(getLogger(), "   • Different thread IDs (multi-threaded mode)");
    RCLCPP_INFO(getLogger(), "   • Overlapping START/END timestamps (concurrency!)");
    RCLCPP_INFO(getLogger(), "   • Same thread ID = single-threaded mode");
    RCLCPP_INFO(getLogger(), "");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "╔════════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(getLogger(), "║      DEMONSTRATION COMPLETE - Check Logs Above!               ║");
    RCLCPP_INFO(getLogger(), "╚════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(getLogger(), "");
  }
};

}  // namespace sm_multithread_test_1
