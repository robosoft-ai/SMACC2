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

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>
#include <config/mission_constants.hpp>

namespace sm_cl_px4_mr_test_4
{

// SUPERSTATE: Victor Sierra sector search #1; the three VS superstates run back to
// back on one datum, each rotated 30 deg from the previous, so together they
// form the classic re-oriented sector search.
// Owns where it is (the pin), its pattern parameters and where the pattern
// starts and ends; the inner run state injects the parameters into the
// behavior.
struct SsVSSearch1 : smacc2::SmaccState<SsVSSearch1, MsInFlight, StiVSSearch1Run>
{
  using SmaccState::SmaccState;

  // P6: ring station 5, shared by the rotated triple
  static NedXY pin() { return ringPinAt(5, 0.0f); }

  static cl_px4_mr::FlightPatternVSSearchParams patternParams()
  {
    const NedXY c = pin();
    cl_px4_mr::FlightPatternVSSearchParams p;
    p.datumX = c.x;
    p.datumY = c.y;
    p.altitudeAgl = kMissionAltitudeM;
    p.radius = kVSRotatedRadiusM;
    p.initialHeading = kVSSearchInitialHeading1;
    p.direction = cl_px4_mr::Turn::RIGHT;
    p.cycles = kVSSearchCycles;
    return p;
  }

  static cl_px4_mr::PathFollowerParams followerParams()
  {
    cl_px4_mr::PathFollowerParams f;
    f.groundSpeed = kPatternSpeedMps;
    f.leash = kPatternLeashM;
    return f;
  }

  // a sector search starts and ends on its datum
  static NedXY entry() { return pin(); }
  static NedXY exit() { return pin(); }

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    const auto p = patternParams();
    RCLCPP_INFO(
      getLogger(), "=== SsVSSearch1: sector search, r=%.0f m, first leg %.0f deg, ~%.0f m, pin P6 ===",
      p.radius, p.initialHeading * 180.0 / M_PI, cl_px4_mr::flightPatternVSSearchLength(p));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsVSSearch1 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
