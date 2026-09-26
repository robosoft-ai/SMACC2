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
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>
#include <cl_px4_mr/client_behaviors/cb_spiral_up.hpp>
#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <cl_px4_mr/components/cp_tunnel_centering.hpp>
#include <cl_px4_mr/utils/geo_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

// Every mission tunable, in one place:
//   1. flight envelope     - altitudes, speeds, leash, tolerances
//   2. lidar gating        - topic, watchdog, safety cone, hold policy
//   3. watchdog timing     - the formula every state derives its timeout from
//   4. the cave            - spawn point and the route, in the gz world frame
//   5. derived helpers     - frame conversion, follower / TF / cone params
//
// Frames: the gz world (Cave Circuit Practice 01) is ENU with x east, y north,
// z up; the cave entrance corridor runs due +x. PX4's local NED origin is the
// spawn point (first GPS fix), so NED north = gz y - spawn.y, east = gz x -
// spawn.x, down = -(gz z - spawn.z). TF frame `map` is that origin, ENU.

namespace sm_cl_px4_mr_test_5
{

using cl_px4_mr::NedPoint;

// ===========================================================================
// 1. Flight envelope
// ===========================================================================
constexpr std::chrono::seconds kInitialPause{5};  // StPause: hold on the ground before starting
// preflight heading gate (StWaitForReady): the parked vehicle faces gz +x = NED
// east; arm only once the EKF heading holds still and agrees with that
constexpr float kSpawnHeadingRad = static_cast<float>(M_PI) / 2.0f;
constexpr double kHeadingGateMaxDriftDegS = 0.5;
constexpr double kHeadingGateWindowS = 5.0;
constexpr double kHeadingGateTimeoutS = 60.0;
constexpr float kHeadingGateTolDeg = 15.0f;
constexpr float kTakeoffAltitudeM = 1.5f;         // CbTakeOff target (above the spawn floor)
constexpr std::chrono::seconds kTakeoffTimeout{40};  // then re-arm and retry (StTakeoff -> StArmPX4)
// route altitude above the spawn floor (gz z = 0.25 + this). The entrance tunnel is
// 5-9 m tall with its floor at gz z 0..0.7 (measured by ray-casting the tile meshes,
// scratchpad cave_profile2.py). The tunnel centering re-trims the height in flight
// (kCenteringFloorClearanceM above the floor, kCenteringCeilingClearanceM below the
// ceiling), so this is the height flown where there is no floor to measure.
constexpr float kCruiseAltitudeM = 2.0f;
constexpr float kClimbRateMps = 1.0f;
constexpr float kCruiseSpeedMps = 1.5f;  // carrot advance rate (slow: 5 m wide passages)
// leash: must exceed 1.2 x (speed / 0.95) = 1.9; short so the vehicle tracks the
// polyline closely instead of cutting corners; it also caps the effective speed
// at ~0.95 x leash if the simulation slows down
constexpr float kCruiseLeashM = 2.5f;
constexpr float kArrivalXyTolM = 0.8f;
constexpr float kArrivalZTolM = 0.5f;
constexpr float kTurnaroundHoldS = 5.0f;  // StHoldAtTurnaround
// pre-landing descent in offboard, holding the spawn point, then AUTO_LAND
constexpr float kPreLandAltitudeM = 0.8f;
constexpr float kPreLandDescentRateMps = 0.5f;
constexpr float kPreLandXyTolM = 0.3f;

// ===========================================================================
// 2. LiDAR gating
// ===========================================================================
constexpr const char * kLidarTopic = "/lidar/points";  // bridged PointCloud2 (frame lidar_link)
constexpr std::chrono::seconds kLidarTimeout{2};       // no cloud for this long -> abort
// forward safety cone (CpForwardObstacleGuard), about the sensor +x axis.
// Keep the trigger range below half the narrowest passage width: the cone
// sweeps the walls during the 180 deg turn at the start of a retrace.
constexpr float kConeHalfAngleDeg = 15.0f;  // narrow enough that the floor 1.3 m below stays out of the trigger range
constexpr float kConeTriggerRangeM = 2.5f;
constexpr float kConeClearRangeM = 3.5f;
constexpr float kConeMinRangeM = 0.6f;  // the airframe's own returns
constexpr float kConeFloorCutoffM = 0.8f;  // returns more than this below the lidar are floor, not obstacle
constexpr int kConeMinHits = 20;
constexpr int kConeTriggerClouds = 2;   // 0.2 s at 10 Hz
constexpr int kConeClearClouds = 10;    // 1 s
constexpr float kConeMaxYawRateRadS = 0.35f;  // clouds taken while yawing faster than this are ignored
// tunnel centering (CpTunnelCentering): the follower adds a lateral offset toward
// the midpoint between the walls and a vertical one holding this clearance above
// the floor, so the route only has to be roughly right
constexpr bool kUseTunnelCentering = true;
constexpr float kCenteringMinRangeM = 0.6f;  // own arms/rotors (same cut as the guard)
constexpr float kCenteringLookAheadMinM = 1.5f;
constexpr float kCenteringLookAheadMaxM = 6.0f;
constexpr float kCenteringFloorClearanceM = 2.0f;  // == kCruiseAltitudeM above a flat floor
constexpr float kCenteringCeilingClearanceM = 1.5f;  // overhead rock: back off early
constexpr float kCenteringMinFloorClearanceM = 1.0f;  // never trimmed closer to the floor than this (run 16 dove into a rock pile)
constexpr float kCenteringColumnHalfWidthM = 1.0f;   // overhead/floor column ~ the airframe; wider reached the walls (run 22 climbed into the arch)
constexpr float kCenteringSmoothing = 0.4f;          // EMA weight of the previous value (lower = quicker)
constexpr float kCenteringFloorLookBackM = 1.5f;      // floor/ceiling column starts this far behind the sensor
constexpr float kCenteringMaxLateralM = 3.0f;
constexpr float kCenteringMaxVerticalM = 1.0f;
constexpr float kCenteringWallFadeM = 6.0f;  // no lateral correction when both walls are farther
constexpr float kObstacleHoldMaxS = 30.0f;  // StObstacleHold: give up waiting after this
constexpr int kMaxObstacleHolds = 3;        // one more -> land where we are

// ===========================================================================
// 3. Watchdog timing (the behaviors' watchdogs run on the wall clock; a slow
//    simulation stretches every leg, hence kRtfMargin)
// ===========================================================================
constexpr float kTimeoutMarginFactor = 2.0f;
constexpr float kTimeoutBaseS = 30.0f;
constexpr float kRtfMargin = 2.0f;

// ===========================================================================
// 4. The cave (gz world frame)
// ===========================================================================
struct GzPoint
{
  float x = 0.0f;  // east
  float y = 0.0f;  // north
  float z = 0.0f;  // local FLOOR height (gz z) at this vertex; the route flies kCruiseAltitudeM above it
};

// where start_cave_sitl.sh spawns the vehicle (SM5_SPAWN_POSE) - keep in sync
constexpr GzPoint kSpawnGz{2.0f, 0.0f, 0.25f};

// Outbound route: a COARSE centreline along y = 0 (the tunnel centering does the
// lateral and vertical trimming in flight), z = local floor height from vertical
// ray casts on the tile meshes (scratchpad cave_band2.py / cave_path_profile.py:
// at floor+2 m the passage is >= 6 m wide around y = 0 everywhere along it). Two legs:
//   A: staging area -> entrance tunnel (tile_1, x 12.5..62.5) -> the shaft bottom
//   B: shaft bottom (x 85-90, the shaft rises ~17 m; the lidar map of run 20 shows
//      it 6-10 m wide, i.e. a tall passage, NOT a room) -> x 92..110 (no mesh floor
//      at y 0) -> portal x 112.5 (ceiling 5.3) -> the ramp of tile_19, floor down
//      to -4.4 m by x 152 and back up to 0 by x 210, passage 5-11 m -> the
//      turnaround at x 210, ~210 m in, short of the corner tile's bend.
// There is no open room on this level: after the bend the corridor runs south
// (x ~237-240) to the top of a 25 m vertical shaft at (237, -150). The
// world's only cavern (tile_44) is at z 75, reachable only up the shafts.
inline std::vector<GzPoint> caveRouteAGz()
{
  return {
    {6.0f, 0.0f, 0.0f},   {12.5f, 0.0f, 0.0f},  {17.5f, 0.0f, 0.0f},  {22.5f, 0.0f, 0.2f},
    {27.5f, 0.0f, 0.6f},  {32.5f, 0.0f, 0.7f},  {37.5f, 0.0f, 0.6f},  {42.5f, 0.0f, 0.3f},
    {47.5f, 0.0f, 0.2f},  {52.5f, 0.0f, 0.1f},  {57.5f, 0.0f, 0.1f},  {62.5f, 0.0f, 0.0f},
    {67.5f, 0.0f, 0.0f},
  };
}

inline std::vector<GzPoint> caveRouteBGz()
{
  return {
    {72.5f, 0.0f, 0.0f},    {77.5f, 0.0f, 0.0f},    {82.5f, 0.0f, 0.2f},    {87.5f, 0.0f, 0.4f},
    {92.5f, 0.0f, 0.0f},    {97.5f, 0.0f, -0.2f},   {102.5f, 0.0f, -0.2f},  {107.5f, 0.0f, -0.2f},
    // the portal into the ramp (x 112-126): the lidar map of run 22 shows the
    // passage at flight height offset south (free y -1..-2.5 at x 116-122, a
    // rock outcrop on the north side down to y -0.5), floors from that map
    {112.5f, -1.0f, -0.9f}, {117.5f, -1.5f, -0.6f}, {122.5f, -1.5f, -1.4f}, {127.5f, -1.0f, -2.5f},
    {132.5f, 0.0f, -3.6f},  {137.5f, 0.0f, -3.4f},  {142.5f, 0.0f, -3.8f},  {147.5f, 0.0f, -4.0f},
    {152.5f, 0.0f, -4.4f},  {157.5f, 0.0f, -4.4f},  {162.5f, 0.0f, -4.0f},  {167.5f, 0.0f, -3.4f},
    {172.5f, 0.0f, -2.9f},  {177.5f, 0.0f, -2.6f},  {182.5f, 0.0f, -2.0f},  {187.5f, 0.0f, -1.4f},
    {192.5f, 0.0f, -0.9f},  {197.5f, 0.0f, -0.5f},  {202.5f, 0.0f, -0.3f},  {207.5f, 0.0f, -0.1f},
    {210.0f, 0.0f, -0.1f},
    // the corridor continues to x ~218, where it bends sharply south-south-east
    // (run 23's lidar map: rock across x 219 from y +1 down to y -4); the trip
    // ends on the straight, short of the bend
  };
}

// Finale over the base station (the SubT tent + antenna at the cave mouth, model
// origin gz (-8, 0) facing south, tent 2 m in front of it). Back at the pad the
// vehicle climbs to the finale altitude, flies to the east point of a circle
// centred on the tent, laps it, spirals up around it, laps it again at the top,
// returns to the pad and lands.
// The staging area has an INVISIBLE collision lid (cave starting area type b,
// fence_link collision_top: a 55 x 60 m box at z 30, walls at y +-30 and x -39.5).
constexpr GzPoint kBaseStationTentGz{-6.0f, 0.0f, 0.0f};
constexpr float kFinaleAltitudeM = 6.0f;  // first laps + helix start; tent ~4 m tall
constexpr float kFinaleRadiusM = 6.0f;    // cone trigger 2.5 m: the tent stays outside it
constexpr float kFinaleAngularRateRadS = 0.3f;  // ~1.8 m/s on the circle
constexpr int kFinaleOrbitsLow = 2;             // level laps before the helix
constexpr float kFinaleClimbPerOrbitM = 1.5f;
constexpr float kFinaleClimbTotalM = 9.0f;  // 6 orbits: 6 -> 15 m, well under the 30 m lid
constexpr float kFinaleTopAltitudeM = kFinaleAltitudeM + kFinaleClimbTotalM;
constexpr int kFinaleOrbitsTop = 2;  // level laps at the top
constexpr float kTurnaroundYawRad = static_cast<float>(M_PI);  // StTurnAround: yaw in place before the retrace
constexpr std::chrono::seconds kTurnTimeout{40};

// ===========================================================================
// 5. Derived helpers - not tunables
// ===========================================================================

// gz world point, flown altitudeM above its local floor height -> PX4 local NED
// (the NED origin is the spawn point, gz z = kSpawnGz.z)
inline NedPoint gzToNed(GzPoint p, float altitudeM)
{
  NedPoint n;
  n.x = p.y - kSpawnGz.y;                          // north
  n.y = p.x - kSpawnGz.x;                          // east
  n.z = -(p.z + altitudeM - kSpawnGz.z);           // down
  return n;
}

inline std::vector<NedPoint> toNedRoute(const std::vector<GzPoint> & gz)
{
  std::vector<NedPoint> route;
  for (const GzPoint & p : gz)
  {
    route.push_back(gzToNed(p, kCruiseAltitudeM));
  }
  return route;
}

inline std::vector<NedPoint> caveRouteANed() { return toNedRoute(caveRouteAGz()); }
inline std::vector<NedPoint> caveRouteBNed() { return toNedRoute(caveRouteBGz()); }

inline NedPoint baseStationTentNed() { return gzToNed(kBaseStationTentGz, kFinaleAltitudeM); }
// where the orbit starts: the point of the circle nearest the pad (due east of the tent)
inline NedPoint finaleOrbitEntryNed()
{
  NedPoint p = baseStationTentNed();
  p.y += kFinaleRadiusM;
  return p;
}
inline cl_px4_mr::SpiralUpParams finaleSpiralParams()
{
  const NedPoint c = baseStationTentNed();
  cl_px4_mr::SpiralUpParams p;
  p.centerX = c.x;
  p.centerY = c.y;
  p.radiusM = kFinaleRadiusM;
  p.startAltitudeM = kFinaleAltitudeM;
  p.climbTotalM = kFinaleClimbTotalM;
  p.climbPerOrbitM = kFinaleClimbPerOrbitM;
  p.angularVelocityRadS = kFinaleAngularRateRadS;
  return p;
}
inline float finaleSpiralLengthM()
{
  return kFinaleClimbTotalM / kFinaleClimbPerOrbitM * 2.0f * static_cast<float>(M_PI) *
         kFinaleRadiusM;
}
inline float finaleOrbitLengthM(int orbits)
{
  return orbits * 2.0f * static_cast<float>(M_PI) * kFinaleRadiusM;
}
inline cl_px4_mr::HeadingStableParams headingGateParams()
{
  cl_px4_mr::HeadingStableParams p;
  p.maxDriftRadS = kHeadingGateMaxDriftDegS * M_PI / 180.0;
  p.windowS = kHeadingGateWindowS;
  p.timeoutS = kHeadingGateTimeoutS;
  p.expectedHeadingRad = kSpawnHeadingRad;
  p.headingTolRad = kHeadingGateTolDeg * static_cast<float>(M_PI) / 180.0f;
  return p;
}

inline NedPoint homeAtCruise() { return gzToNed(kSpawnGz, kCruiseAltitudeM); }

inline float routeLengthM(const std::vector<NedPoint> & path)
{
  float len = 0.0f;
  for (size_t i = 1; i < path.size(); ++i)
  {
    len += cl_px4_mr::nedDistance(path[i - 1], path[i]);
  }
  return len;
}

// the traversed prefix of a route, flown backwards, then home at cruise altitude
inline std::vector<NedPoint> retracePath(const std::vector<NedPoint> & traversed)
{
  std::vector<NedPoint> back(traversed.rbegin(), traversed.rend());
  back.push_back(homeAtCruise());
  return back;
}

inline cl_px4_mr::PathFollowerParams cruiseFollower()
{
  cl_px4_mr::PathFollowerParams f;
  f.groundSpeed = kCruiseSpeedMps;
  f.leash = kCruiseLeashM;
  f.arrivalXyTol = kArrivalXyTolM;
  f.arrivalZTol = kArrivalZTolM;
  f.yawMode = cl_px4_mr::YawMode::TANGENT;  // nose along the path: the cone looks ahead
  f.prependCurrentPosition = true;
  f.autoTimeoutFactor = 0.0f;  // every state arms its own watchdog
  f.useTunnelCentering = kUseTunnelCentering;
  return f;
}

inline cl_px4_mr::TunnelCenteringParams centeringParams()
{
  cl_px4_mr::TunnelCenteringParams p;
  p.lookAheadMinM = kCenteringLookAheadMinM;
  p.lookAheadMaxM = kCenteringLookAheadMaxM;
  p.floorClearanceM = kCenteringFloorClearanceM;
  p.ceilingClearanceM = kCenteringCeilingClearanceM;
  p.maxLateralM = kCenteringMaxLateralM;
  p.maxVerticalM = kCenteringMaxVerticalM;
  p.wallFadeM = kCenteringWallFadeM;
  p.minFloorClearanceM = kCenteringMinFloorClearanceM;
  p.columnHalfWidthM = kCenteringColumnHalfWidthM;
  p.floorLookBackM = kCenteringFloorLookBackM;
  p.smoothing = kCenteringSmoothing;
  p.minRangeM = kCenteringMinRangeM;
  return p;
}

inline cl_px4_mr::TfBroadcasterParams tfParams()
{
  cl_px4_mr::TfBroadcasterParams p;  // map -> base_link / base_link_follow, static base_link -> lidar_link
  p.sensorXyz[0] = 0.12;             // lidar include pose (0.12, 0, 0.26) + sensor pose z 0.055
  p.sensorXyz[1] = 0.0;
  p.sensorXyz[2] = 0.315;
  return p;
}

inline cl_px4_mr::ForwardObstacleGuardParams coneParams()
{
  cl_px4_mr::ForwardObstacleGuardParams p;
  p.halfAngleRad = kConeHalfAngleDeg * static_cast<float>(M_PI) / 180.0f;
  p.triggerRangeM = kConeTriggerRangeM;
  p.clearRangeM = kConeClearRangeM;
  p.minRangeM = kConeMinRangeM;
  p.floorCutoffM = kConeFloorCutoffM;
  p.minHits = kConeMinHits;
  p.triggerClouds = kConeTriggerClouds;
  p.clearClouds = kConeClearClouds;
  p.maxYawRateRadS = kConeMaxYawRateRadS;
  return p;
}

// timeout = (length / speed * margin + base) * rtf margin
inline std::chrono::seconds legTimeout(float lengthM, float speedMps)
{
  const float seconds =
    (lengthM / std::max(speedMps, 0.1f) * kTimeoutMarginFactor + kTimeoutBaseS) * kRtfMargin;
  return std::chrono::seconds(static_cast<long>(std::ceil(seconds)));
}

}  // namespace sm_cl_px4_mr_test_5
