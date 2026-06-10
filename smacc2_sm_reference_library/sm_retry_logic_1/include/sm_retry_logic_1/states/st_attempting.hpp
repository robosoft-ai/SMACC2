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
// StAttempting: The task is running with a 10s timeout.
// Timer expiry or 'f' keypress counts as a failure (inner retry, back to StIdle).
// The SrEventCountdown in SsAttemptTask is notified for both EvTimer and EvKeyPressF.
// Press 's' to succeed — unhandled here, propagates up to SsAttemptTask (→ StSuccess).
struct StAttempting : smacc2::SmaccState<StAttempting, SsAttemptTask>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StIdle>,
    Transition<EvKeyPressF<CbDefaultKeyboardBehavior, OrKeyboard>, StIdle>
    > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StAttempting] Attempt running. Press 'f' to fail, 's' to succeed, or wait 10s."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[StAttempting] Attempt ended."); }
};
}  // namespace sm_retry_logic_1
