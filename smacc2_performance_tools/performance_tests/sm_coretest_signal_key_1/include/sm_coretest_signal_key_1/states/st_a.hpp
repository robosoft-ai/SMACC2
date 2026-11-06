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
// StA owns the mixin-first subscriber for 1 s. When StA exits, the subscriber is disposed
// and destroyed; the tick component keeps firing at 5 Hz into StRest.
struct StA : smacc2::SmaccState<StA, SmCoretestSignalKey1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StRest>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(1s);
    configure_orthogonal<OrTick, CbMixinFirstSubscriber>();
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StA] Entered. Subscriber alive for 1s."); }
  void onExit() { RCLCPP_INFO(getLogger(), "[StA] Exiting."); }
};
}  // namespace sm_coretest_signal_key_1
