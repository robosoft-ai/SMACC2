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
// SsAttemptTask: Superstate that owns a SrEventCountdown(3) reactor.
// The reactor listens for timer timeouts (EvTimer from StAttempting) and
// manual failure keypresses ('f', EvKeyPressF from StAttempting).
// After 3 such failures the reactor fires EvCountdownEnd, transitioning to StExhausted.
// Pressing 's' at any time (StIdle or StAttempting) propagates up to this superstate
// and transitions to StSuccess.
struct SsAttemptTask : smacc2::SmaccState<SsAttemptTask, SmRetryLogic1, StIdle>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCountdownEnd<SrEventCountdown, SsAttemptTask>, StExhausted>,
    Transition<EvKeyPressS<CbDefaultKeyboardBehavior, OrKeyboard>, StSuccess>
    > reactions;

  static void staticConfigure()
  {
    static_createStateReactor<
      SrEventCountdown,
      EvCountdownEnd<SrEventCountdown, SsAttemptTask>,
      mpl::list<
        EvTimer<CbTimerCountdownOnce, OrTimer>,
        EvKeyPressF<CbDefaultKeyboardBehavior, OrKeyboard>>>(3);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[SsAttemptTask] Entered. 3 attempts allowed."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[SsAttemptTask] Exiting."); }
};
}  // namespace sm_retry_logic_1
