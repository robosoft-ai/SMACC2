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

// SUPERSTATE: Victor Sierra sector search #2 at its pin. The three VS
// superstates run consecutively, each rotated 30 deg from the previous, so
// together they form the classic re-oriented sector search.
struct SsVSSearch2 : smacc2::SmaccState<SsVSSearch2, MsInFlight, StiVSSearch2Run>
{
  using SmaccState::SmaccState;

  struct Params
  {
    cl_px4_mr::FlightPatternVSSearchParams pattern = railway::vsSearchParams(2);
    cl_px4_mr::PathFollowerParams follower = railway::patternFollowerParams();
  } params;

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "=== SsVSSearch2: sector search, r=%.0f m, first leg %.0f deg, ~%.0f m ===",
      params.pattern.radius, params.pattern.initialHeading * 180.0 / M_PI,
      cl_px4_mr::flightPatternVSSearchLength(params.pattern));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsVSSearch2 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
