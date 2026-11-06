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

namespace sm_coretest_signal_key_1
{
// No subscriber here. Any "[CbMixinFirstSubscriber] TICK" line logged while StRest is
// active is a callback into a destroyed behavior.
struct StRest : smacc2::SmaccState<StRest, SmCoretestSignalKey1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, StA>> reactions;

  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownLoop>(1s); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StRest] Entered. No subscriber; ticks must stop."); }
  void onExit() { RCLCPP_INFO(getLogger(), "[StRest] Exiting."); }
};
}  // namespace sm_coretest_signal_key_1
