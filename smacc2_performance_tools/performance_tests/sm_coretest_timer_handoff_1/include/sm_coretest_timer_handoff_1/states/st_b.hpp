// Copyright 2026 RobosoftAI Inc.
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
struct StB : smacc2::SmaccState<StB, SmCoretestTimerHandoff1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, StA>> reactions;

  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownLoop>(100ms); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StB] Entered."); }
};
}  // namespace sm_coretest_timer_handoff_1
