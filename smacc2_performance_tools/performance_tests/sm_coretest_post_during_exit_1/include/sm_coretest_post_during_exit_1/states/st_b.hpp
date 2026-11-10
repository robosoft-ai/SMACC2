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

namespace sm_coretest_post_during_exit_1
{
struct StB : smacc2::SmaccState<StB, SmCoretestPostDuringExit1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvCbSuccess<CbSleepFor, OrSleep>, StA>> reactions;

  static void staticConfigure() { configure_orthogonal<OrSleep, CbSleepFor>(50ms); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StB] Entered."); }
};
}  // namespace sm_coretest_post_during_exit_1
