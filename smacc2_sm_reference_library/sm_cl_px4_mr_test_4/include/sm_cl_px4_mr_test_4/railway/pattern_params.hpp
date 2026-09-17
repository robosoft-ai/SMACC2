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

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>

#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_plan.hpp>

#include <cmath>
#include <vector>

// One source of truth for every pattern's parameters. The superstates copy
// these into their behaviors (with the pin as origin); the planner uses the
// same parameters to compute each pattern's entry and exit points so transit
// legs fly straight to where the pattern starts, not to its centroid.

namespace sm_cl_px4_mr_test_4
{
namespace railway
{

inline cl_px4_mr::PathFollowerParams patternFollowerParams()
{
  cl_px4_mr::PathFollowerParams f;
  f.groundSpeed = kPatternSpeedMps;
  f.leash = kPatternLeashM;
  return f;
}

inline cl_px4_mr::FlightPatternSquareSpiralParams squareSpiral1Params()
{
  cl_px4_mr::FlightPatternSquareSpiralParams p;
  p.direction = cl_px4_mr::Turn::RIGHT;
  p.altitudeAgl = kMissionAltitudeM;
  p.initialHeading = kSquareSpiralInitialHeading;
  p.spacing = kTrackSpacingM;
  p.numLegs = kSquareSpiralNumLegs;
  return p;
}

inline cl_px4_mr::FlightPatternSquareSpiralParams squareSpiral2Params()
{
  cl_px4_mr::FlightPatternSquareSpiralParams p = squareSpiral1Params();
  p.direction = cl_px4_mr::Turn::LEFT;
  p.numLegs = kSquareSpiral2NumLegs;
  return p;
}

inline cl_px4_mr::FlightPatternLawnmowerParams lawnmower1Params()
{
  cl_px4_mr::FlightPatternLawnmowerParams p;
  p.laneHeading = kLawnmowerHeadingA;
  p.firstTurn = kLawnmowerFirstTurnA;
  p.altitudeAgl = kMissionAltitudeM;
  p.laneLength = kPatternSquareSideM;
  p.width = kPatternSquareSideM;
  p.laneSpacing = kTrackSpacingM;
  p.originIsCenter = true;
  return p;
}

inline cl_px4_mr::FlightPatternLawnmowerParams lawnmower2Params()
{
  cl_px4_mr::FlightPatternLawnmowerParams p = lawnmower1Params();
  p.laneHeading = kLawnmowerHeadingB;
  p.firstTurn = kLawnmowerFirstTurnB;
  return p;
}

inline cl_px4_mr::FlightPatternGridPatternParams gridPattern1Params()
{
  cl_px4_mr::FlightPatternGridPatternParams p;
  p.base.laneHeading = kGridHeading;
  p.base.firstTurn = kGridFirstTurn;
  p.base.altitudeAgl = kMissionAltitudeM;
  p.base.laneLength = kPatternSquareSideM;
  p.base.width = kPatternSquareSideM;
  p.base.laneSpacing = kTrackSpacingM;
  p.base.originIsCenter = true;
  p.secondPass = true;
  return p;
}

// the rotating triple: n = 1, 2, 3
inline cl_px4_mr::FlightPatternVSSearchParams vsSearchParams(int n)
{
  cl_px4_mr::FlightPatternVSSearchParams p;
  p.altitudeAgl = kMissionAltitudeM;
  p.radius = kVSRotatedRadiusM;
  p.initialHeading = n == 1 ? kVSSearchInitialHeading1
                            : (n == 2 ? kVSSearchInitialHeading2 : kVSSearchInitialHeading3);
  p.direction = cl_px4_mr::Turn::RIGHT;
  p.cycles = kVSSearchCycles;
  return p;
}

// pearls on a chain (all identical)
inline cl_px4_mr::FlightPatternVSSearchParams vsChainParams()
{
  cl_px4_mr::FlightPatternVSSearchParams p;
  p.altitudeAgl = kMissionAltitudeM;
  p.radius = kVSSearchRadiusM;
  p.initialHeading = kVSChainInitialHeading;
  p.direction = cl_px4_mr::Turn::RIGHT;
  p.cycles = kVSSearchCycles;
  return p;
}

// CbFigureEight (parametric) configuration for figure-eight n = 1, 2
struct FigureEightSpec
{
  float altitudeAgl;
  float size;      // half-length (centre to lobe tip)
  float speed;     // parameter rate, rad/s
  int loops;
  float heading;   // lobe axis, NED yaw
};

inline FigureEightSpec figureEightSpec(int n)
{
  FigureEightSpec s;
  s.altitudeAgl = kMissionAltitudeM;
  s.size = kFigureEightHalfLengthM;
  s.speed = kFigureEightSpeedRad;
  s.loops = kFigureEightLoops;
  s.heading = n == 1 ? kFigureEightHeading1 : kFigureEightHeading2;
  return s;
}

inline cl_px4_mr::FlightPatternLoiterParams centroidLoiterParams()
{
  cl_px4_mr::FlightPatternLoiterParams p;
  p.altitudeAgl = kMissionAltitudeM;
  p.radius = kLoiterRadiusM;
  p.count = kCentroidLoiterCount;
  return p;
}

// ---------------------------------------------------------------------------
// Where a pattern built around `pin` starts and ends. Transit legs target the
// entry; leg lengths (timeouts) run from the previous pattern's exit.
struct PatternEndpoints
{
  cl_px4_mr::NedPoint entry;
  cl_px4_mr::NedPoint exit;
};

inline PatternEndpoints patternEndpoints(NodeRole role, float pinX, float pinY, float altitudeNedZ)
{
  using namespace cl_px4_mr;
  NedPoint pin;
  pin.x = pinX;
  pin.y = pinY;
  pin.z = altitudeNedZ;
  pin.yaw = 0.0f;  // nominal entry heading for generators that default to it

  PatternEndpoints e;
  e.entry = pin;
  e.exit = pin;
  auto fromPath = [&](const std::vector<NedPoint> & path) {
    if (!path.empty())
    {
      e.entry = path.front();
      e.exit = path.back();
    }
  };

  switch (role)
  {
    case NodeRole::PIN_SQUARE_SPIRAL_1:
    {
      auto p = squareSpiral1Params();
      p.originX = pinX;
      p.originY = pinY;
      fromPath(generateFlightPatternSquareSpiral(p, pin));
      break;
    }
    case NodeRole::PIN_SQUARE_SPIRAL_2:
    {
      auto p = squareSpiral2Params();
      p.originX = pinX;
      p.originY = pinY;
      fromPath(generateFlightPatternSquareSpiral(p, pin));
      break;
    }
    case NodeRole::PIN_LAWNMOWER_1:
    case NodeRole::PIN_LAWNMOWER_2:
    {
      auto p = role == NodeRole::PIN_LAWNMOWER_1 ? lawnmower1Params() : lawnmower2Params();
      p.originX = pinX;
      p.originY = pinY;
      fromPath(generateFlightPatternLawnmower(p, pin));
      break;
    }
    case NodeRole::PIN_GRID_PATTERN_1:
    {
      auto p = gridPattern1Params();
      p.base.originX = pinX;
      p.base.originY = pinY;
      fromPath(generateFlightPatternGridPattern(p, pin));
      break;
    }
    case NodeRole::FIGURE_EIGHT_1:
    case NodeRole::FIGURE_EIGHT_2:
    {
      // CbFigureEight starts (and ends) at the lobe tip along its axis
      const FigureEightSpec s = figureEightSpec(role == NodeRole::FIGURE_EIGHT_1 ? 1 : 2);
      e.entry.x = pinX + s.size * std::cos(s.heading);
      e.entry.y = pinY + s.size * std::sin(s.heading);
      e.exit = e.entry;
      break;
    }
    default:
      // sector searches, loiters: start and end on the datum / centroid
      break;
  }
  return e;
}

}  // namespace railway
}  // namespace sm_cl_px4_mr_test_4
