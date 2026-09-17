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

#include <cl_px4_mr/utils/pattern_generators.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/pattern_params.hpp>

namespace sm_cl_px4_mr_test_4
{

// SUPERSTATE: crosshatch grid search at its backbone pin. Holds the pattern parameters in one
// place; the inner run state injects them into the behavior together with the
// pin read from the mission plan.
struct SsGridPattern1 : smacc2::SmaccState<SsGridPattern1, MsInFlight, StiGridPattern1Run>
{
  using SmaccState::SmaccState;

  struct Params
  {
    cl_px4_mr::FlightPatternGridPatternParams pattern = railway::gridPattern1Params();
    cl_px4_mr::PathFollowerParams follower = railway::patternFollowerParams();
  } params;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "=== SsGridPattern1: crosshatch grid search, ~%.0f m ===", cl_px4_mr::flightPatternGridPatternLength(params.pattern));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsGridPattern1 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
