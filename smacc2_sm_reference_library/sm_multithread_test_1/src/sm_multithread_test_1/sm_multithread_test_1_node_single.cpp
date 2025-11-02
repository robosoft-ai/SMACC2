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
 * @file sm_multithread_test_1_node_single.cpp
 * @brief Single-threaded variant for comparison
 *
 * This file runs the same state machine in single-threaded mode for comparison.
 * The only difference from sm_multithread_test_1_node.cpp is the execution model.
 *
 * COMPARISON:
 *
 * Multi-threaded (sm_multithread_test_1_node.cpp):
 *   smacc2::run<SmMultithreadTest1>(smacc2::ExecutionModel::MULTI_THREAD_SPINNER);
 *
 * Single-threaded (this file):
 *   smacc2::run<SmMultithreadTest1>(smacc2::ExecutionModel::SINGLE_THREAD_SPINNER);
 *   or simply:
 *   smacc2::run<SmMultithreadTest1>();  // SINGLE_THREAD_SPINNER is default
 *
 * EXPECTED BEHAVIOR DIFFERENCES:
 *
 * Single-threaded mode:
 * - All callbacks execute on same thread (same thread ID in logs)
 * - Serial execution (one callback at a time)
 * - No overlap in START/END timestamps
 * - More predictable/deterministic
 *
 * Multi-threaded mode:
 * - Different thread IDs for concurrent callbacks
 * - Parallel execution (multiple callbacks simultaneously)
 * - Overlapping START/END timestamps
 * - Higher throughput for I/O-bound operations
 */

#include <sm_multithread_test_1/sm_multithread_test_1.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ╔════════════════════════════════════════════════════════════════════╗
  // ║  🔄 SINGLE-THREADED EXECUTOR (Default Mode)                       ║
  // ║                                                                    ║
  // ║  This uses the traditional single-threaded executor for           ║
  // ║  comparison with the multi-threaded variant.                      ║
  // ║                                                                    ║
  // ║  Callbacks execute serially on a single thread.                   ║
  // ╚════════════════════════════════════════════════════════════════════╝

  smacc2::run<sm_multithread_test_1::SmMultithreadTest1>(
    smacc2::ExecutionModel::SINGLE_THREAD_SPINNER  // Traditional mode
  );

  // Alternative (equivalent) way - SINGLE_THREAD_SPINNER is the default:
  // smacc2::run<sm_multithread_test_1::SmMultithreadTest1>();

  return 0;
}
