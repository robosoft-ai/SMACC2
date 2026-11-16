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

#include <smacc2/smacc.hpp>

namespace sm_nav2_gazebo_test_4
{
// Terminal state reached from any behavior failure or safety timer.
struct StMissionFailed : smacc2::SmaccState<StMissionFailed, SmNav2GazeboTest4>
{
  using SmaccState::SmaccState;

  // clang-format off
  typedef mpl::list<> reactions;
  // clang-format on

  static void staticConfigure() {}

  void onEntry() { RCLCPP_ERROR(getLogger(), "StMissionFailed: a behavior failed or timed out"); }
};

}  // namespace sm_nav2_gazebo_test_4
