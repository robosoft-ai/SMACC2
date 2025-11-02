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

namespace sm_multithread_test_1
{

/**
 * @brief Terminal state - demonstration is complete
 *
 * This simple state marks the end of the multi-threaded executor demonstration.
 * The state machine will remain in this state until manually terminated.
 */
struct StComplete : smacc2::SmaccState<StComplete, SmMultithreadTest1>
{
  using SmaccState::SmaccState;

  // No transitions - this is a terminal state
  typedef mpl::list<> reactions;

  static void staticConfigure()
  {
    // No behaviors needed in terminal state
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "StComplete::runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "════════════════════════════════════════════════════════════════");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "  ✅ Multi-threaded Executor Demonstration Complete!");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "  📊 Review the logs above to observe:");
    RCLCPP_INFO(getLogger(), "     • Thread IDs in callback execution");
    RCLCPP_INFO(getLogger(), "     • Concurrent vs serial execution patterns");
    RCLCPP_INFO(getLogger(), "     • Overlapping work timestamps");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "  🔄 To compare modes, run:");
    RCLCPP_INFO(getLogger(), "     • Multi-threaded:  ros2 launch sm_multithread_test_1 sm_multithread_test_1.launch.py");
    RCLCPP_INFO(getLogger(), "     • Single-threaded: ros2 launch sm_multithread_test_1 sm_multithread_test_1_single.launch.py");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "  Press Ctrl+C to exit.");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "════════════════════════════════════════════════════════════════");
    RCLCPP_INFO(getLogger(), "");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StComplete::onExit()");
  }
};

}  // namespace sm_multithread_test_1
