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

namespace sm_cl_px4_mr_test_4
{

using namespace smacc2::default_transition_tags;

// STATE: terminal - the mission could not be loaded, vehicle never armed
struct StMissionAborted : smacc2::SmaccState<StMissionAborted, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_ERROR(getLogger(), "StMissionAborted: no mission backbone available - MISSION ABORTED");
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
