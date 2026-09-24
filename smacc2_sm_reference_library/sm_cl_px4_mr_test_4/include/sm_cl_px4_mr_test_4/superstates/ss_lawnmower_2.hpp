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

// SUPERSTATE: lawnmower, lanes north-south (orientation B), stepping west.
// Owns where it is (the pin), its pattern parameters and where the pattern
// starts and ends; the inner run state injects the parameters into the
// behavior.
struct SsLawnmower2 : smacc2::SmaccState<SsLawnmower2, MsInFlight, StiLawnmower2Run>
{
  using SmaccState::SmaccState;

  // P4: ring station 3
  static NedXY pin() { return ringPinAt(3, 0.0f); }

  static cl_px4_mr::FlightPatternLawnmowerParams patternParams()
  {
    const NedXY c = pin();
    cl_px4_mr::FlightPatternLawnmowerParams p;
    p.originX = c.x;
    p.originY = c.y;
    p.laneHeading = kLawnmowerHeadingB;
    p.firstTurn = kLawnmowerFirstTurnB;
    p.altitudeAgl = kMissionAltitudeM;
    p.laneLength = kPatternSquareSideM;
    p.width = kPatternSquareSideM;
    p.laneSpacing = kTrackSpacingM;
    p.originIsCenter = true;
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
    return pathEntry(cl_px4_mr::generateFlightPatternLawnmower(patternParams(), patternOrigin(pin())), pin());
  }
  static NedXY exit()
  {
    return pathExit(cl_px4_mr::generateFlightPatternLawnmower(patternParams(), patternOrigin(pin())), pin());
  }

  typedef mpl::list<
  > reactions;

  static void staticConfigure() {}
  void runtimeConfigure() {}

  void onEntry()
  {
    const auto p = patternParams();
    RCLCPP_INFO(
      getLogger(), "=== SsLawnmower2: lawnmower, lanes north-south (orientation B), ~%.0f m, pin P4 ===",
      cl_px4_mr::flightPatternLawnmowerLength(p));
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "=== Exiting SsLawnmower2 ===");
  }
};

}  // namespace sm_cl_px4_mr_test_4
