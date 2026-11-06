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

namespace sm_coretest_forced_finish_1
{
struct StA : smacc2::SmaccState<StA, SmCoretestForcedFinish1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCountdownEnd<SrEventCountdown, StA>, StRest>,
    // The 30 s sleep cannot finish within the 1 s StA lifetime, so a sleep event
    // processed here is a leftover that crossed a previous StA exit.
    Transition<EvCbSuccess<CbSleepFor, OrSleep>, StTrap>,
    Transition<EvCbFinished<CbSleepFor, OrSleep>, StTrap>
    > reactions;

  static void staticConfigure()
  {
    // 30 s sleep against a 1 s timer: the sleep can never finish on its own.
    configure_orthogonal<OrSleep, CbSleepFor>(rclcpp::Duration(30s));
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(1s);

    // One EvTimer ends the countdown; the reactor is disposed with the state
    // (exercises the state-reactor loop in notifyOnStateExited).
    static_createStateReactor<
      SrEventCountdown,
      EvCountdownEnd<SrEventCountdown, StA>,
      mpl::list<EvTimer<CbTimerCountdownOnce, OrTimer>>>(1);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StA] Entered. Sleep 30s vs timer 1s."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[StA] Exiting."); }
};
}  // namespace sm_coretest_forced_finish_1
