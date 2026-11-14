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

namespace sm_coretest_action_client_1
{
// A goal of 1000 steps against a 500 ms timer: the state always exits with the goal in flight, and
// the behavior's onExit cancels it. Neither behavior event may be seen here or in the next state.
struct StCancelled : smacc2::SmaccState<StCancelled, SmCoretestActionClient1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StSuccess>,
    Transition<EvCbSuccess<CbFibonacci, OrAction>, StTrap>,
    Transition<EvCbFailure<CbFibonacci, OrAction>, StTrap>>
    reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrAction, CbFibonacci>(1000);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(500ms);
  }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StCancelled] Entered."); }
};
}  // namespace sm_coretest_action_client_1
