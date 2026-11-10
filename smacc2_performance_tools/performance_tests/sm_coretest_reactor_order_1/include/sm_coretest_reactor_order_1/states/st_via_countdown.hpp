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

namespace sm_coretest_reactor_order_1
{
struct StViaCountdown : smacc2::SmaccState<StViaCountdown, SmCoretestReactorOrder1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvX, StA>> reactions;

  static void staticConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(),
      "[StViaCountdown] Entered: EvCountdownEnd was processed before its trigger EvX.");
  }
};
}  // namespace sm_coretest_reactor_order_1
