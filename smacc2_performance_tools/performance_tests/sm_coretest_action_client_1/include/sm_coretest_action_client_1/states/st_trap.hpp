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
// Terminal. Entered only when a state saw the outcome it must not see.
struct StTrap : smacc2::SmaccState<StTrap, SmCoretestActionClient1>
{
  using SmaccState::SmaccState;

  void onEntry() { RCLCPP_FATAL(getLogger(), "[StTrap] TRAP ENTERED."); }
};
}  // namespace sm_coretest_action_client_1
