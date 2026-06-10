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

namespace sm_retry_logic_1
{
// StIdle: Initial inner state of SsAttemptTask.
// Press 'n' to start an attempt (→ StAttempting).
// Press 's' to succeed — unhandled here, propagates up to SsAttemptTask (→ StSuccess).
// Remaining failure count is preserved in SsAttemptTask's SrEventCountdown across retries.
struct StIdle : smacc2::SmaccState<StIdle, SsAttemptTask>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAttempting>
    > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StIdle] Waiting. Press 'n' to attempt, 's' to succeed."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[StIdle] Starting attempt."); }
};
}  // namespace sm_retry_logic_1
