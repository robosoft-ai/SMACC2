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

namespace sm_coretest_timer_handoff_1
{
// The forced-finish cycle at 100 ms: the sleep is cut short by the timer on every exit, so the state
// machine thread holds the exit for the forced-finish wait and then creates the next state's timer; the
// executor in the polling loop has to pick that timer up or the machine never leaves StB.
struct StA : smacc2::SmaccState<StA, SmCoretestTimerHandoff1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvCountdownEnd<SrEventCountdown, StA>, StB>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrSleep, CbSleepFor>(rclcpp::Duration(30s));
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(100ms);
    static_createStateReactor<
      SrEventCountdown, EvCountdownEnd<SrEventCountdown, StA>,
      mpl::list<EvTimer<CbTimerCountdownOnce, OrTimer>>>(1);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StA] Entered."); }
};
}  // namespace sm_coretest_timer_handoff_1
