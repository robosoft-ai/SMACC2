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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <cl_px4_mr/utils/geo_utils.hpp>

#include <cmath>
#include <limits>
#include <vector>

// Pure trajectory-pattern generators: params + the vehicle's entry point in,
// an ordered NED polyline out. No ROS, no components - unit-testable and
// reusable by any follower.
//
// Naming convention: every pattern carries the FlightPattern prefix, e.g. for
// a pattern called Lawnmower:
//   struct FlightPatternLawnmowerParams { ... };
//   std::vector<NedPoint> generateFlightPatternLawnmower(
//     const FlightPatternLawnmowerParams &, const NedPoint & current);
//   float flightPatternLawnmowerLength(const FlightPatternLawnmowerParams &);  // watchdog sizing
// The generator must stay pure: no ROS, no components, no clock.
//
// Conventions:
//   - NED: x north, y east, z down (altitude params are positive metres AGL,
//     converted to z = -altitude)
//   - NaN origin / altitude / heading = "use the entry point's value"
//   - Turn::RIGHT = clockwise viewed from above = increasing yaw
//   - yaw radians, NaN = leave to the follower's yaw mode

namespace cl_px4_mr
{

enum class Turn
{
  RIGHT,
  LEFT
};

inline float turnSign(Turn t) { return t == Turn::RIGHT ? 1.0f : -1.0f; }
inline const char * turnName(Turn t) { return t == Turn::RIGHT ? "RIGHT" : "LEFT"; }

namespace pattern_detail
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
inline float pick(float param, float fallback) { return std::isnan(param) ? fallback : param; }
inline float altitudeToZ(float altitudeAgl, float currentZ)
{
  return std::isnan(altitudeAgl) ? currentZ : -altitudeAgl;
}
}  // namespace pattern_detail

// ---------------------------------------------------------------------------
// Rate-controlled vertical move to an altitude (no horizontal motion)
struct FlightPatternAscendParams
{
  float altitudeAgl = 10.0f;  // target, metres AGL
  float climbRate = 1.5f;     // m/s, becomes the follower ground speed
  float tolerance = 0.5f;     // already within this -> single-vertex path (immediate success)
  // XY to hold during the climb/descent; NaN = the entry position. Set it to
  // land precisely on a point (pre-landing descent).
  float targetX = pattern_detail::kNaN;
  float targetY = pattern_detail::kNaN;
};
std::vector<NedPoint> generateFlightPatternAscend(const FlightPatternAscendParams & p, const NedPoint & current);

// ---------------------------------------------------------------------------
// N circles about a centre, entered at the nearest point
struct FlightPatternLoiterParams
{
  float centerX = pattern_detail::kNaN;
  float centerY = pattern_detail::kNaN;
  float altitudeAgl = pattern_detail::kNaN;
  float radius = 5.0f;
  int count = 1;  // 0 -> no circles: single vertex at the entry point, immediate success
  Turn direction = Turn::RIGHT;
  int pointsPerCircle = 36;
  bool faceCenter = true;  // per-vertex yaw toward the centre; else tangent
};
std::vector<NedPoint> generateFlightPatternLoiter(const FlightPatternLoiterParams & p, const NedPoint & current);

// ---------------------------------------------------------------------------
// Straight transit with sinusoidal altitude about the base altitude
struct FlightPatternSineWaveVerticalParams
{
  float endX = pattern_detail::kNaN;
  float endY = pattern_detail::kNaN;
  float baseAltitudeAgl = pattern_detail::kNaN;
  float amplitude = 1.5f;
  float wavelength = 20.0f;
  float sampleSpacing = 1.0f;
};
std::vector<NedPoint> generateFlightPatternSineWaveVertical(
  const FlightPatternSineWaveVerticalParams & p, const NedPoint & current);

// ---------------------------------------------------------------------------
// Straight transit with sinusoidal lateral weave about the leg line
struct FlightPatternSineWaveHorizontalParams
{
  float endX = pattern_detail::kNaN;
  float endY = pattern_detail::kNaN;
  float altitudeAgl = pattern_detail::kNaN;
  float amplitude = 3.0f;
  float wavelength = 20.0f;
  float sampleSpacing = 1.0f;
};
std::vector<NedPoint> generateFlightPatternSineWaveHorizontal(
  const FlightPatternSineWaveHorizontalParams & p, const NedPoint & current);

// ---------------------------------------------------------------------------
// Expanding square spiral: legs s, s, 2s, 2s, 3s, 3s, ... with 90 deg turns
struct FlightPatternSquareSpiralParams
{
  Turn direction = Turn::RIGHT;
  float originX = pattern_detail::kNaN;
  float originY = pattern_detail::kNaN;
  float altitudeAgl = pattern_detail::kNaN;
  float initialHeading = pattern_detail::kNaN;  // first leg heading; NaN = entry heading
  float spacing = 10.0f;                        // track spacing (m)
  int numLegs = 12;
  float maxLegLength = 0.0f;  // 0 = unbounded; otherwise stop before a longer leg
};
std::vector<NedPoint> generateFlightPatternSquareSpiral(const FlightPatternSquareSpiralParams & p, const NedPoint & current);
float flightPatternSquareSpiralLength(const FlightPatternSquareSpiralParams & p);

// ---------------------------------------------------------------------------
// Archimedean spiral r = (spacing / 2 pi) * theta between two radii
struct FlightPatternSpiralParams
{
  float centerX = pattern_detail::kNaN;
  float centerY = pattern_detail::kNaN;
  float altitudeAgl = pattern_detail::kNaN;
  float startRadius = 0.0f;
  float endRadius = 50.0f;
  float spacing = 10.0f;  // radial gap between successive turns (m)
  Turn direction = Turn::RIGHT;
  bool inward = false;         // fly from endRadius in to startRadius
  float sampleSpacing = 2.0f;  // arc length between vertices (m)
};
std::vector<NedPoint> generateFlightPatternSpiral(const FlightPatternSpiralParams & p, const NedPoint & current);
float flightPatternSpiralLength(const FlightPatternSpiralParams & p);

// ---------------------------------------------------------------------------
// Boustrophedon lawnmower: parallel lanes along `laneHeading`, stepping
// sideways by laneSpacing across `width`. With originIsCenter the origin is
// the centre of the covered rectangle (pins are area centres); otherwise it is
// the start corner of the first lane.
struct FlightPatternLawnmowerParams
{
  float originX = pattern_detail::kNaN;
  float originY = pattern_detail::kNaN;
  bool originIsCenter = true;
  float altitudeAgl = pattern_detail::kNaN;
  float laneHeading = pattern_detail::kNaN;  // NED yaw of the lanes; NaN = entry heading
  float laneLength = 50.0f;
  float width = 30.0f;
  float laneSpacing = 10.0f;
  Turn firstTurn = Turn::RIGHT;  // side the lanes step toward
};
std::vector<NedPoint> generateFlightPatternLawnmower(const FlightPatternLawnmowerParams & p, const NedPoint & current);
float flightPatternLawnmowerLength(const FlightPatternLawnmowerParams & p);
int flightPatternLawnmowerLaneCount(const FlightPatternLawnmowerParams & p);

// ---------------------------------------------------------------------------
// Crosshatch grid: a lawnmower pass, then a second pass rotated 90 degrees
// over the same rectangle, starting from the corner nearest the end of pass 1
struct FlightPatternGridPatternParams
{
  FlightPatternLawnmowerParams base;
  bool secondPass = true;
};
std::vector<NedPoint> generateFlightPatternGridPattern(const FlightPatternGridPatternParams & p, const NedPoint & current);
float flightPatternGridPatternLength(const FlightPatternGridPatternParams & p);

// ---------------------------------------------------------------------------
// Victor Sierra (IAMSAR sector search): nine legs of length `radius` from a
// datum with 120 degree turns; the craft passes back through the datum after
// legs 3, 6 and 9, covering three 60 degree sectors. Extra cycles rotate the
// pattern by `reorientation` (classic 30 degrees) to fill the gaps.
struct FlightPatternVSSearchParams
{
  float datumX = pattern_detail::kNaN;
  float datumY = pattern_detail::kNaN;
  float altitudeAgl = pattern_detail::kNaN;
  float radius = 30.0f;
  float initialHeading = pattern_detail::kNaN;  // first leg heading; NaN = entry heading
  Turn direction = Turn::RIGHT;
  int cycles = 1;
  float reorientation = static_cast<float>(M_PI) / 6.0f;
};
std::vector<NedPoint> generateFlightPatternVSSearch(const FlightPatternVSSearchParams & p, const NedPoint & current);
float flightPatternVSSearchLength(const FlightPatternVSSearchParams & p);

// heading (NED yaw) of the leg from `from` to `to`
inline float legHeading(const NedPoint & from, const NedPoint & to)
{
  return std::atan2(to.y - from.y, to.x - from.x);
}

}  // namespace cl_px4_mr
