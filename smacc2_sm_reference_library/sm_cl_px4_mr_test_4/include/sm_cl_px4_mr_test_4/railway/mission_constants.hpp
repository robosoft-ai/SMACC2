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

#include <cl_px4_mr/utils/geo_utils.hpp>
#include <cl_px4_mr/utils/pattern_generators.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// Every mission tunable, in one place, in four blocks:
//   1. flight envelope            - altitudes, speeds, leashes, watchdogs
//   2. shared pattern geometry    - the two master values every pattern derives from
//   3. DEMO_RING layout           - the demo, in the order it is flown
//   4. BACKBONE layout + infra    - the Hormuz mission, KML, test-leg support
// This build hardcodes them; a later phase may populate the same values from
// ROS 2 parameters or a config file without touching states or behaviors.
// Values marked "derived" are computed from the masters and are not tunables.

namespace sm_cl_px4_mr_test_4
{
namespace railway
{

using cl_px4_mr::Turn;
constexpr float kPi = static_cast<float>(M_PI);

// ===========================================================================
// 1. Flight envelope
// ===========================================================================
constexpr std::chrono::seconds kInitialPause{5};  // StPause: hold on the ground before starting
constexpr float kTakeoffAltitudeM = 5.0f;         // CbTakeOff target (AGL)
constexpr float kMissionAltitudeM = 30.0f;        // cruise / pattern altitude (AGL)
constexpr float kAscendClimbRateMps = 1.5f;       // StAscend, takeoff altitude -> mission altitude
constexpr float kCruiseSpeedMps = 5.0f;           // transit legs
constexpr float kPatternSpeedMps = 3.0f;          // pattern superstates
// carrot leash (m): must exceed the tracking lag (speed / 0.95) with margin
constexpr float kCruiseLeashM = 10.0f;
constexpr float kPatternLeashM = 6.0f;
// watchdogs: timeout = path length / speed * factor + base
constexpr float kTimeoutMarginFactor = 2.0f;
constexpr float kTimeoutBaseS = 30.0f;
// pre-landing descent in offboard, holding the landing point, then AUTO_LAND
constexpr float kPreLandAltitudeM = 1.0f;        // hand over to PX4 land mode from here
constexpr float kPreLandDescentRateMps = 1.0f;
constexpr float kPreLandXyTolM = 0.3f;           // arrival tolerance over the pad

// ===========================================================================
// 2. Shared pattern geometry
// ===========================================================================
// Track spacing (m) between adjacent passes of every coverage pattern: the
// spiral's turn spacing, the square-spiral track spacing, the lawnmower and
// grid lane spacing. Change this one value to tighten or loosen the whole demo.
constexpr float kTrackSpacingM = 10.0f;
// Side (m) of the square footprint every pattern covers: the square spirals'
// outer extent, the lawnmower/grid rectangles, the sector-search diameter,
// the figure-eight length.
constexpr float kPatternSquareSideM = 180.0f;
// loiter circles (landing zone and the figure-eight centroid)
constexpr float kLoiterRadiusM = 10.0f;

// ===========================================================================
// 3. DEMO_RING layout, in flight order
// ===========================================================================
enum class Layout
{
  DEMO_RING,  // pins on a ring around the takeoff point (~3 h demo)
  BACKBONE    // pins interpolated along the Hormuz backbone (~48 km)
};
constexpr Layout kLayout = Layout::DEMO_RING;

// --- ring ---
constexpr float kDemoRingRadiusM = 550.0f;     // pins on this circle about the takeoff point
constexpr float kDemoRingStartBearing = 0.0f;  // first pin due north (NED yaw, rad)
constexpr bool kDemoRingClockwise = true;      // visiting direction around the ring

// --- spiral off the island (StSpiralOffIsland, flown before the ring) ---
constexpr float kSpiralOffIslandTurns = 8.5f;
constexpr Turn kSpiralOffIslandDirection = Turn::RIGHT;

// --- transit nav states between pins ---
constexpr float kSineAmplitudeM = 5.0f;
constexpr float kSineWavelengthM = 100.0f;

// --- P1 / P2: square spirals ---
constexpr float kSquareSpiralInitialHeading = 0.0f;  // first leg due north
// legs come in pairs of equal length s, s, 2s, 2s, ...; the outer extent is
// spacing * numLegs / 2, so 36 legs at 10 m fill the 180 m square
constexpr int kSquareSpiralFullLegs =
  static_cast<int>(2.0f * kPatternSquareSideM / kTrackSpacingM);  // derived: 36
// SsSquareSpiral1 (RIGHT): one leg short, so it ends at its south-east corner,
// the corner nearest P2
constexpr int kSquareSpiralNumLegs = kSquareSpiralFullLegs - 1;   // 35
// SsSquareSpiral2 (LEFT): a full revolution more than the first
constexpr int kSquareSpiral2NumLegs = kSquareSpiralFullLegs + 4;  // 40

// --- P3 / P4: lawnmowers (180 m square, 19 lanes at the track spacing) ---
// A: lanes flown east -> west (first lane heading west), pattern progressing
//    north -> south: step LEFT of a westbound heading = south, so the start
//    corner is the north-east one, nearest the approach from P2
constexpr float kLawnmowerHeadingA = -kPi / 2.0f;
constexpr Turn kLawnmowerFirstTurnA = Turn::LEFT;
// B: lanes north-south, first lane heading south, stepping RIGHT of
//    southbound = west (start corner north-east)
constexpr float kLawnmowerHeadingB = kPi;
constexpr Turn kLawnmowerFirstTurnB = Turn::RIGHT;

// --- P5: crosshatch grid (180 m square, two passes) ---
constexpr float kGridHeading = 0.0f;  // first pass N-S, second pass E-W
// first pass starts at the south-east corner and steps west (LEFT of a
// northbound heading), so a vehicle arriving from the east does not overfly
// the grid area to reach the start corner
constexpr Turn kGridFirstTurn = Turn::LEFT;

// --- P6: rotating sector-search triple (one datum, 0 / 30 / 60 deg) ---
constexpr float kVSSearchRadiusM = kPatternSquareSideM / 2.0f;  // derived: 90 m, covers the square width
constexpr float kVSRotatedRadiusM = 1.15f * kVSSearchRadiusM;   // the rotating triple, 15 % larger
constexpr float kVSSearchInitialHeading1 = 0.0f;
constexpr float kVSSearchInitialHeading2 = kPi / 6.0f;
constexpr float kVSSearchInitialHeading3 = kPi / 3.0f;
constexpr int kVSSearchCycles = 1;

// --- P7: sector-search pearls (three datums along the ring tangent) ---
constexpr float kDemoChainSpacingM = kPatternSquareSideM;  // 2 x radius: the circles touch
constexpr float kVSChainInitialHeading = 0.0f;

// --- tail: back to the square-spiral centre, south past the island, then
//     figure-eights and a loiter at a centroid, then land at the island ---
// SouthWaypoint: planner-owned NED offset from the takeoff origin. The
// surveyed position 26.477152 N, 56.538247 E projects to 205.4 m S / 8.6 m W
// of the island origin (26.478999 N, 56.538333 E); the demo waypoint sits
// 50 m closer to the origin along that same bearing (155.6 m out).
constexpr float kSouthWaypointSouthM = 155.5f;
constexpr float kSouthWaypointWestM = 6.5f;
constexpr float kFigureEightOffsetSouthM = 100.0f;  // centroid this far south of SouthWaypoint
constexpr float kFigureEightHalfLengthM = kPatternSquareSideM / 2.0f;  // derived: 180 m long
constexpr float kFigureEightHeading1 = 0.0f;       // lobes north-south
constexpr float kFigureEightHeading2 = kPi / 2.0f;  // rotated 90 deg
constexpr int kFigureEightLoops = 2;
// derived: CbFigureEight's `speed` is the lemniscate parameter rate (rad/s);
// peak ground speed = halfLength * speed, so this holds the pattern speed
constexpr float kFigureEightSpeedRad = kPatternSpeedMps / kFigureEightHalfLengthM;
constexpr int kCentroidLoiterCount = 3;

// ===========================================================================
// 4. BACKBONE layout and infrastructure
// ===========================================================================
// where the mission ends: the island (takeoff point) or the hotel (backbone
// vertex B4, 42 km away - only sensible with Layout::BACKBONE)
enum class LandingSite
{
  ISLAND,
  HOTEL
};
constexpr LandingSite kLandingSite = LandingSite::ISLAND;
// loiter over the landing zone before landing; the demo already loitered at
// the figure-eight centroid, so only the backbone mission does
constexpr int kLandingLoiterCount = kLayout == Layout::BACKBONE ? 3 : 0;
// pins sit at k/(N+1) of the backbone arc length; node gaps longer than this
// are subdivided so the sine-wave transits get more anchor points
constexpr float kInterpolationSpacingM = 3500.0f;

// --- KML backbone (loaded at startup in every layout) ---
inline const char * kPackageName = "sm_cl_px4_mr_test_4";
inline const char * kKmlFileName = "Hormuz_3.kml";

// Compiled-in backbone: used only when config/Hormuz_3.kml is absent or
// unparseable. Must match the KML's LineString vertices.
struct BackboneVertex
{
  const char * name;
  double lat;
  double lon;
};

inline const std::vector<BackboneVertex> & fallbackBackboneVertices()
{
  static const std::vector<BackboneVertex> vertices = {
    {"TakeOff", 26.478986, 56.538518},
    {"StraitWaypoint1", 26.400958, 56.318406},
    {"GridAreaApproach", 26.210850, 56.256207},
    {"Harbor", 26.211050, 56.243102},
    {"LandingZoneHotel", 26.213792, 56.233753},
  };
  return vertices;
}

inline std::vector<cl_px4_mr::GeoPoint> fallbackBackbone()
{
  std::vector<cl_px4_mr::GeoPoint> points;
  for (const auto & v : fallbackBackboneVertices())
  {
    points.push_back(cl_px4_mr::GeoPoint{v.lat, v.lon, 0.0});
  }
  return points;
}

// Role name for backbone vertex i (falls back to "B<i>" beyond the known list)
inline std::string backboneName(size_t i)
{
  const auto & vertices = fallbackBackboneVertices();
  if (i < vertices.size())
  {
    return vertices[i].name;
  }
  return "B" + std::to_string(i);
}

// --- isolated testing (test_leg) ---
constexpr float kTestLegSyntheticDistanceM = 150.0f;  // transit target for nav-state test legs

}  // namespace railway
}  // namespace sm_cl_px4_mr_test_4
