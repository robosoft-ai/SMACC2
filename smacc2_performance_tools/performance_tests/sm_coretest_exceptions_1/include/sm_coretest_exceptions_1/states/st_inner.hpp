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
// Inside SsEscalate, with no reaction of its own to a failed initialization. Its runtimeConfigure
// (site inner_configure) or its onEntry (site inner_entry) throws.
struct StInner : smacc2::SmaccState<StInner, SsEscalate>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StRetry>> reactions;

  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownOnce>(500ms); }

  void runtimeConfigure() { throwIfSite(getLogger(), "inner_configure"); }

  void onEntry()
  {
    throwIfSite(getLogger(), "inner_entry");
    RCLCPP_INFO(getLogger(), "[StInner] Entered.");
  }

  void onExit() { RCLCPP_INFO(getLogger(), "[StInner] Exiting."); }
};
}  // namespace sm_coretest_exceptions_1
