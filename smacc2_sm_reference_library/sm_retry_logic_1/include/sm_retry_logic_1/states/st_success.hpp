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

namespace sm_retry_logic_1
{
// StSuccess: Terminal state reached when 's' is pressed during any attempt.
struct StSuccess : smacc2::SmaccState<StSuccess, SmRetryLogic1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<> reactions;

  void onEntry() { RCLCPP_INFO(getLogger(), "[StSuccess] Task succeeded!"); }
};
}  // namespace sm_retry_logic_1
