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
// The goal goes to an action nothing serves; sendGoal posts the failure without waiting.
struct StNoServer : smacc2::SmaccState<StNoServer, SmCoretestActionClient1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbFailure<CbFibonacciNoServer, OrAction>, StCancelled>,
    Transition<EvCbSuccess<CbFibonacciNoServer, OrAction>, StTrap>>
    reactions;

  static void staticConfigure() { configure_orthogonal<OrAction, CbFibonacciNoServer>(); }

  void onEntry() { RCLCPP_INFO(getLogger(), "[StNoServer] Entered."); }
};
}  // namespace sm_coretest_action_client_1
