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

// SUPERSTATE: expanding square spiral (RIGHT); one leg short of a full fill so it
// ends at its south-east corner, the one nearest P2.
// Owns where it is (the pin), its pattern parameters and where the pattern
// starts and ends; the inner run state injects the parameters into the
// behavior.
struct SsSquareSpiral1 : smacc2::SmaccState<SsSquareSpiral1, MsInFlight, StiSquareSpiral1Run>
{
  using SmaccState::SmaccState;

  // P1: ring station 0 (due north)
  static NedXY pin() { return ringPinAt(0, 0.0f); }

  static cl_px4_mr::FlightPatternSquareSpiralParams patternParams()
  {
    const NedXY c = pin();
    cl_px4_mr::FlightPatternSquareSpiralParams p;
    p.originX = c.x;
    p.originY = c.y;
    p.direction = cl_px4_mr::Turn::RIGHT;
    p.altitudeAgl = kMissionAltitudeM;
    p.initialHeading = kSquareSpiralInitialHeading;
    p.spacing = kTrackSpacingM;
    p.numLegs = kSquareSpiralNumLegs;
    return p;
  }

  static cl_px4_mr::PathFollowerParams followerParams()
  {
    cl_px4_mr::PathFollowerParams f;
    f.groundSpeed = kPatternSpeedMps;
    f.leash = kPatternLeashM;
    return f;
  }

  // where the pattern starts / ends: the next transit flies to entry(), its
  // leg length runs from exit()
  static NedXY entry()
  {
    return pathEntry(cl_px4_mr::generateFlightPatternSquareSpiral(patternParams(), patternOrigin(pin())), pin());
  }
  static NedXY exit()
  {
    return pathExit(cl_px4_mr::generateFlightPatternSquareSpiral(patternParams(), patternOrigin(pin())), pin());
  }

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    const auto p = patternParams();
    RCLCPP_INFO(
      getLogger(), "=== SsSquareSpiral1: %s square spiral, spacing %.0f m, %d legs, pin P1 ===",
      cl_px4_mr::turnName(p.direction), p.spacing, p.numLegs);
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsSquareSpiral1 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
