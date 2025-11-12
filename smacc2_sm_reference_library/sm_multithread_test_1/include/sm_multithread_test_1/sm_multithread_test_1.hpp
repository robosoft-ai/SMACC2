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

/**
 * @file sm_multithread_test_1.hpp
 * @brief Multi-threaded Executor Demonstration State Machine
 *
 * This state machine demonstrates SMACC2's multi-threaded executor capability
 * (ExecutionModel::MULTI_THREAD_SPINNER) by running multiple concurrent timer
 * callbacks with simulated work.
 *
 * Key Features Demonstrated:
 * - Multi-threaded ROS2 callback execution
 * - Concurrent timer processing
 * - Thread ID logging to prove parallelism
 * - Work simulation to make concurrency observable
 *
 * Addresses GitHub Issue #571:
 * https://github.com/robosoft-ai/SMACC2/issues/571
 *
 * @see README.md for detailed usage instructions and architecture explanation
 */

#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

// ORTHOGONALS
#include "orthogonals/or_timer_a.hpp"
#include "orthogonals/or_timer_b.hpp"
#include "orthogonals/or_timer_c.hpp"
#include "orthogonals/or_timer_d.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;

namespace sm_multithread_test_1
{

// STATE FORWARD DECLARATIONS
class StConcurrentOperation;
class StComplete;

//═══════════════════════════════════════════════════════════════════════════
// STATE MACHINE
//═══════════════════════════════════════════════════════════════════════════

/**
 * @brief SmMultithreadTest1 - Demonstrates multi-threaded executor usage
 *
 * This state machine is designed to showcase the difference between
 * single-threaded and multi-threaded execution models in SMACC2.
 *
 * Architecture:
 * - 4 orthogonals, each with a timer client at different rates
 * - Custom behaviors that simulate work and log thread IDs
 * - Visual demonstration of concurrent vs serial execution
 *
 * Initial State: StConcurrentOperation
 * - Runs 4 concurrent timers for 15 seconds
 * - Logs thread IDs and work simulation
 * - Transitions to StComplete after 15 seconds
 *
 * Terminal State: StComplete
 * - Displays summary and instructions
 * - Waits for user to terminate
 *
 * Usage:
 * Multi-threaded:  ros2 launch sm_multithread_test_1 sm_multithread_test_1.launch.py
 * Single-threaded: ros2 launch sm_multithread_test_1 sm_multithread_test_1_single.launch.py
 */
struct SmMultithreadTest1
  : public smacc2::SmaccStateMachineBase<SmMultithreadTest1, StConcurrentOperation>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "╔════════════════════════════════════════════════════════════════╗");
    RCLCPP_INFO(getLogger(), "║    SmMultithreadTest1 - Multi-threaded Executor Demo         ║");
    RCLCPP_INFO(getLogger(), "╚════════════════════════════════════════════════════════════════╝");
    RCLCPP_INFO(getLogger(), "");
    RCLCPP_INFO(getLogger(), "Initializing state machine with 4 timer orthogonals...");

    // Create the four orthogonals, each with a timer at a different rate
    this->createOrthogonal<OrTimerA>();  // 100ms
    this->createOrthogonal<OrTimerB>();  // 250ms
    this->createOrthogonal<OrTimerC>();  // 500ms
    this->createOrthogonal<OrTimerD>();  // 1000ms

    RCLCPP_INFO(getLogger(), "State machine initialization complete.");
    RCLCPP_INFO(getLogger(), "");
  }
};

}  // namespace sm_multithread_test_1

// STATE INCLUDES (must be after state machine declaration)
#include "states/st_concurrent_operation.hpp"
#include "states/st_complete.hpp"
