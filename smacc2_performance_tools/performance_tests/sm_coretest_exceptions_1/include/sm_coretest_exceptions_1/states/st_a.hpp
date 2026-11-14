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
// The state every throw site lives in: its own onEntry and onExit, and the three behaviors.
struct StA : smacc2::SmaccState<StA, SmCoretestExceptions1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StRest>> reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(500ms);
    configure_orthogonal<OrThrow, CbSyncThrow>();
    configure_orthogonal<OrThrow, CbAsyncThrow>();
    configure_orthogonal<OrThrow, CbUpdateThrow>();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StA] Entered.");
    throwIfSite(getLogger(), "state_entry");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StA] Exiting.");
    throwIfSite(getLogger(), "state_exit");
  }
};
}  // namespace sm_coretest_exceptions_1
