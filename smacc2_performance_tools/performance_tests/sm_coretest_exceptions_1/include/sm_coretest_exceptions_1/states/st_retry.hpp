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

namespace sm_coretest_exceptions_1
{
// Reacts to its own failed initialization by entering itself again. With the site retry_entry its
// onEntry throws on two entries out of three, so every third attempt gets through.
struct StRetry : smacc2::SmaccState<StRetry, SmCoretestExceptions1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownLoop, OrTimer>, StA>,
    Transition<EvStInitializationFailure<StRetry>, StRetry>>
    reactions;

  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownLoop>(500ms); }

  void onEntry()
  {
    static int attempts = 0;
    if (++attempts % 3 != 0) throwIfSite(getLogger(), "retry_entry");
    RCLCPP_INFO(getLogger(), "[StRetry] Entered.");
  }
};
}  // namespace sm_coretest_exceptions_1
