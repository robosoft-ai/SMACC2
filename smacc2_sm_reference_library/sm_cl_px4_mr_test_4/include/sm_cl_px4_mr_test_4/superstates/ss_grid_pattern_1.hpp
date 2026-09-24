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

// SUPERSTATE: crosshatch grid search, two passes; the first starts at the south-east
// corner so the approach from the east does not overfly the area.
// Owns where it is (the pin), its pattern parameters and where the pattern
// starts and ends; the inner run state injects the parameters into the
// behavior.
struct SsGridPattern1 : smacc2::SmaccState<SsGridPattern1, MsInFlight, StiGridPattern1Run>
{
  using SmaccState::SmaccState;

  // P5: ring station 4
  static NedXY pin() { return ringPinAt(4, 0.0f); }

  static cl_px4_mr::FlightPatternGridPatternParams patternParams()
  {
    const NedXY c = pin();
    cl_px4_mr::FlightPatternGridPatternParams p;
    p.base.originX = c.x;
    p.base.originY = c.y;
    p.base.laneHeading = kGridHeading;
    p.base.firstTurn = kGridFirstTurn;
    p.base.altitudeAgl = kMissionAltitudeM;
    p.base.laneLength = kPatternSquareSideM;
    p.base.width = kPatternSquareSideM;
    p.base.laneSpacing = kTrackSpacingM;
    p.base.originIsCenter = true;
    p.secondPass = true;  // crosshatch: second pass at 90 deg
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
    return pathEntry(cl_px4_mr::generateFlightPatternGridPattern(patternParams(), patternOrigin(pin())), pin());
  }
  static NedXY exit()
  {
    return pathExit(cl_px4_mr::generateFlightPatternGridPattern(patternParams(), patternOrigin(pin())), pin());
  }

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    const auto p = patternParams();
    RCLCPP_INFO(
      getLogger(), "=== SsGridPattern1: crosshatch grid search, ~%.0f m, pin P5 ===",
      cl_px4_mr::flightPatternGridPatternLength(p));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsGridPattern1 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
