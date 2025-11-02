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
 * @file sm_multithread_test_1_node.cpp
 * @brief Main entry point for multi-threaded executor demonstration
 *
 * 🎯 KEY FEATURE DEMONSTRATED HERE 🎯
 *
 * This file shows how to enable multi-threaded execution in SMACC2.
 * The only change from a standard SMACC2 state machine is passing
 * ExecutionModel::MULTI_THREAD_SPINNER to smacc2::run().
 *
 * COMPARISON:
 *
 * Standard (single-threaded) SMACC2 state machine:
 *   smacc2::run<MyStateMachine>();
 *
 * Multi-threaded SMACC2 state machine:
 *   smacc2::run<MyStateMachine>(smacc2::ExecutionModel::MULTI_THREAD_SPINNER);
 *                                ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *                                THIS IS THE ONLY CHANGE NEEDED!
 *
 * WHAT THIS ENABLES:
 * - Concurrent ROS2 callback processing
 * - Multiple timer callbacks executing simultaneously
 * - Parallel subscriber callbacks
 * - Parallel action client callbacks
 *
 * LIMITATIONS:
 * - ISmaccUpdatable::update() is NOT called in multi-threaded mode
 * - Components/behaviors relying on update() won't work
 * - Use ROS2 callback patterns instead (timers, subscribers, etc.)
 *
 * See README.md for detailed explanation and usage guide.
 */

#include <sm_multithread_test_1/sm_multithread_test_1.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ╔════════════════════════════════════════════════════════════════════╗
  // ║  🚀 MULTI-THREADED EXECUTOR ENABLED                                ║
  // ║                                                                    ║
  // ║  The second parameter enables ROS2's MultiThreadedExecutor,       ║
  // ║  allowing concurrent callback processing.                         ║
  // ║                                                                    ║
  // ║  Remove the parameter (or use SINGLE_THREAD_SPINNER) for          ║
  // ║  traditional single-threaded execution.                           ║
  // ╚════════════════════════════════════════════════════════════════════╝

  smacc2::run<sm_multithread_test_1::SmMultithreadTest1>(
    smacc2::ExecutionModel::MULTI_THREAD_SPINNER  // ← THE KEY FEATURE!
  );

  return 0;
}
