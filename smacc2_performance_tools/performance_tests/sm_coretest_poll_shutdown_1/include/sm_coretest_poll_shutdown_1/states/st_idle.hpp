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

namespace sm_coretest_poll_shutdown_1
{
// Single state, no transitions. The state machine thread is idle; only the polling thread
// works, which is the thread under test.
struct StIdle : smacc2::SmaccState<StIdle, SmCoretestPollShutdown1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<> reactions;

  static void staticConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "[StIdle] Entered."); }
  void onExit() { RCLCPP_INFO(getLogger(), "[StIdle] Exiting."); }
};
}  // namespace sm_coretest_poll_shutdown_1
