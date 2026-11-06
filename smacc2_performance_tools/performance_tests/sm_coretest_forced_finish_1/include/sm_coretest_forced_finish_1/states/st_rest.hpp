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
struct StRest : smacc2::SmaccState<StRest, SmCoretestForcedFinish1>
{
  using SmaccState::SmaccState;

  // A LOOP timer, deliberately a different behavior type than StA's ONCE timer:
  // StA's EvTimer<CbTimerCountdownOnce, OrTimer> is still queued behind the
  // reactor's EvCountdownEnd when StRest is entered, and would otherwise fire
  // StRest's reaction immediately (the "events firing in the next state" rule).
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, StA>,
    // Either sleep event here crossed the StA exit instead of being discarded.
    Transition<EvCbSuccess<CbSleepFor, OrSleep>, StTrap>,
    Transition<EvCbFinished<CbSleepFor, OrSleep>, StTrap>
    > reactions;

  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownLoop>(1s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StRest] Entered. Timer 1s."); }

  void onExit() { RCLCPP_INFO(getLogger(), "[StRest] Exiting."); }
};
}  // namespace sm_coretest_forced_finish_1
