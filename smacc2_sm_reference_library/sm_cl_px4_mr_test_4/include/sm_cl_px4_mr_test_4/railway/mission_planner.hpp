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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cl_px4_mr/utils/geo_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_plan.hpp>
#include <sm_cl_px4_mr_test_4/railway/pattern_params.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

// Mission planning logic owned by the railway: the state machine (not the
// KML) computes everything above bare backbone waypoints - projection to the
// PX4 local frame, pin placement at k/7 of the arc length, backbone
// densification, leg generation, and the single-leg test plans.

namespace sm_cl_px4_mr_test_4
{
namespace railway
{

// lat/lon (deg) -> NED x/y (m); returns false when no reference is available.
// MsInFlight binds the local-position component's projectToNed; tests bind a
// bare cl_px4_mr::MapProjection.
using Projector = std::function<bool(double lat, double lon, float & x, float & y)>;
// NED x/y (m) -> lat/lon (deg), for logging; may be empty.
using Reprojector = std::function<bool(float x, float y, double & lat, double & lon)>;

// absolute path of config/<kKmlFileName> in this package's share directory
inline std::string kmlSharePath()
{
  try
  {
    return ament_index_cpp::get_package_share_directory(kPackageName) + "/config/" + kKmlFileName;
  }
  catch (const std::exception &)
  {
    return std::string("/nonexistent/") + kKmlFileName;
  }
}

inline std::chrono::seconds transitTimeout(float lengthM)
{
  const float seconds = lengthM / kCruiseSpeedMps * kTimeoutMarginFactor + kTimeoutBaseS;
  return std::chrono::seconds(static_cast<long>(std::ceil(seconds)));
}

// watchdog for a pattern of known path length flown at `speed`
inline std::chrono::seconds patternTimeout(float pathLengthM, float speed)
{
  const float seconds = pathLengthM / std::max(speed, 0.1f) * kTimeoutMarginFactor + kTimeoutBaseS;
  return std::chrono::seconds(static_cast<long>(std::ceil(seconds)));
}

namespace detail
{

struct Anchor
{
  std::string name;
  cl_px4_mr::GeoPoint geo;
  float x = 0.0f;
  float y = 0.0f;
  double s = 0.0;  // arc length from B0 (m)
  NodeRole role = NodeRole::BACKBONE;
};

inline cl_px4_mr::GeoPoint lerpGeo(
  const cl_px4_mr::GeoPoint & a, const cl_px4_mr::GeoPoint & b, double t)
{
  return cl_px4_mr::GeoPoint{
    a.lat + t * (b.lat - a.lat), a.lon + t * (b.lon - a.lon), a.alt + t * (b.alt - a.alt)};
}

// Point at arc length s along the anchor chain, projected to NED
inline bool anchorAtArcLength(
  const std::vector<Anchor> & chain, double s, const Projector & project, Anchor & out)
{
  for (size_t j = 0; j + 1 < chain.size(); ++j)
  {
    if (s >= chain[j].s && s <= chain[j + 1].s)
    {
      const double span = chain[j + 1].s - chain[j].s;
      const double t = span > 0.0 ? (s - chain[j].s) / span : 0.0;
      out.geo = lerpGeo(chain[j].geo, chain[j + 1].geo, t);
      out.s = s;
      return project(out.geo.lat, out.geo.lon, out.x, out.y);
    }
  }
  return false;
}

// fill entry/exit for every node: pattern nodes from their generators, plain
// nodes at their own position
inline void annotateEndpoints(MissionPlan & plan)
{
  for (MissionNode & n : plan.nodes)
  {
    n.entryX = n.x;
    n.entryY = n.y;
    n.exitX = n.x;
    n.exitY = n.y;
    if (isPinRole(n.role))
    {
      const PatternEndpoints e = patternEndpoints(n.role, n.x, n.y, plan.altitudeNedZ);
      n.entryX = e.entry.x;
      n.entryY = e.entry.y;
      n.exitX = e.exit.x;
      n.exitY = e.exit.y;
    }
  }
}

inline void emitLegs(MissionPlan & plan)
{
  annotateEndpoints(plan);
  int transitOrdinal = 0;
  int index = 0;
  for (size_t i = 1; i < plan.nodes.size(); ++i)
  {
    const MissionNode & from = plan.nodes[i - 1];
    const MissionNode & to = plan.nodes[i];
    // transit runs from where the previous pattern ended to where this one starts
    const float length = std::hypot(to.entryX - from.exitX, to.entryY - from.exitY);

    MissionLeg leg;
    leg.fromNode = static_cast<int>(i - 1);
    leg.toNode = static_cast<int>(i);
    leg.lengthM = length;

    // co-located pattern nodes (e.g. the rotated sector-search triple share
    // one datum): no transit, the next pattern starts where the last ended
    const bool coLocated = std::hypot(to.x - from.x, to.y - from.y) < 1.0f && isPinRole(to.role);

    if (to.role == NodeRole::LANDING_ZONE)
    {
      leg.index = index++;
      leg.kind = LegKind::LANDING;
      leg.label = from.name + " -> " + to.name;
      plan.legs.push_back(leg);
      continue;
    }

    if (!coLocated)
    {
      leg.index = index++;
      if (to.approach)
      {
        leg.kind = *to.approach;
      }
      else
      {
        leg.kind = (transitOrdinal++ % 2 == 0) ? LegKind::TRANSIT_V : LegKind::TRANSIT_H;
      }
      leg.label = from.name + " -> " + to.name;
      plan.legs.push_back(leg);
    }

    if (isPinRole(to.role))
    {
      MissionLeg pattern;
      pattern.index = index++;
      pattern.kind = LegKind::PATTERN;
      pattern.role = to.role;
      pattern.fromNode = static_cast<int>(i);
      pattern.toNode = static_cast<int>(i);
      pattern.lengthM = 0.0f;
      pattern.label = std::string("@") + to.name;
      plan.legs.push_back(pattern);
    }
  }
}

}  // namespace detail

// Full mission: backbone (lat/lon) -> nodes/legs in the PX4 local frame.
inline MissionPlan buildMissionPlan(
  const std::vector<cl_px4_mr::GeoPoint> & backbone, const Projector & project, float altitudeM,
  const std::string & source, rclcpp::Logger logger)
{
  using detail::Anchor;

  MissionPlan plan;
  plan.altitudeNedZ = -altitudeM;
  plan.source = source;

  if (backbone.size() < 2)
  {
    RCLCPP_ERROR(logger, "[planner] backbone has %zu points (need >= 2)", backbone.size());
    return plan;
  }

  // 1. backbone anchors: project + arc length
  std::vector<Anchor> chain;
  for (size_t i = 0; i < backbone.size(); ++i)
  {
    Anchor a;
    a.name = backboneName(i);
    a.geo = backbone[i];
    if (!project(a.geo.lat, a.geo.lon, a.x, a.y))
    {
      RCLCPP_ERROR(
        logger, "[planner] projection failed for backbone vertex %zu (no global reference?)", i);
      return plan;
    }
    a.s = i == 0 ? 0.0 : chain.back().s + cl_px4_mr::haversineDistance(chain.back().geo, a.geo);
    a.role = i == 0 ? NodeRole::TAKEOFF
                    : (i + 1 == backbone.size() ? NodeRole::LANDING_ZONE : NodeRole::BACKBONE);
    chain.push_back(a);
  }
  const double total = chain.back().s;

  // 2. pins at k/(N+1) of the arc length
  std::vector<Anchor> anchors = chain;
  const int pinDivisor = kNumPinRoles + 1;
  for (int k = 1; k <= kNumPinRoles; ++k)
  {
    Anchor pin;
    pin.name = "W" + std::to_string(k);
    pin.role = pinRole(k);
    if (!detail::anchorAtArcLength(chain, total * k / pinDivisor, project, pin))
    {
      RCLCPP_ERROR(logger, "[planner] could not place pin W%d", k);
      return plan;
    }
    anchors.push_back(pin);
  }
  std::stable_sort(anchors.begin(), anchors.end(), [](const Anchor & a, const Anchor & b) {
    return a.s < b.s;
  });

  // 3. densify gaps longer than kInterpolationSpacingM
  std::vector<Anchor> dense;
  int interpIndex = 1;
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    if (i > 0)
    {
      const double gap = anchors[i].s - anchors[i - 1].s;
      const int n = std::max(1, static_cast<int>(std::ceil(gap / kInterpolationSpacingM)));
      for (int k = 1; k < n; ++k)
      {
        Anchor mid;
        mid.name = "I" + std::to_string(interpIndex++);
        mid.role = NodeRole::BACKBONE;
        if (!detail::anchorAtArcLength(chain, anchors[i - 1].s + gap * k / n, project, mid))
        {
          RCLCPP_ERROR(logger, "[planner] could not densify gap before %s", anchors[i].name.c_str());
          return plan;
        }
        dense.push_back(mid);
      }
    }
    dense.push_back(anchors[i]);
  }

  // 4. nodes + legs
  for (const Anchor & a : dense)
  {
    MissionNode n;
    n.name = a.name;
    n.geo = a.geo;
    n.x = a.x;
    n.y = a.y;
    n.arcLengthM = a.s;
    n.role = a.role;
    plan.nodes.push_back(n);
  }
  plan.landingNodeIndex = static_cast<int>(plan.nodes.size()) - 1;
  detail::emitLegs(plan);
  plan.valid = !plan.legs.empty();

  RCLCPP_INFO(
    logger, "[planner] backbone %zu vertices, total arc %.0f m, %zu nodes, %zu legs",
    backbone.size(), total, plan.nodes.size(), plan.legs.size());
  return plan;
}

// Demo layout: the superstate pins on a ring of kDemoRingRadiusM about the
// takeoff point, visited in kPinRoles order, transits on the chords between
// them, landing back at the island (or at backbone vertex B4 for HOTEL).
inline MissionPlan buildDemoRingPlan(
  const std::vector<cl_px4_mr::GeoPoint> & backbone, const Projector & project,
  const Reprojector & reproject, float altitudeM, LandingSite landingSite,
  const std::string & source, rclcpp::Logger logger)
{
  MissionPlan plan;
  plan.altitudeNedZ = -altitudeM;
  plan.source = "demo ring (backbone: " + source + ")";

  auto geoOf = [&reproject](float x, float y) {
    cl_px4_mr::GeoPoint g;
    if (reproject)
    {
      reproject(x, y, g.lat, g.lon);
    }
    return g;
  };

  MissionNode home;
  home.name = "Island";
  home.role = NodeRole::TAKEOFF;
  home.geo = geoOf(0.0f, 0.0f);
  plan.nodes.push_back(home);

  // Ring stops: each superstate role sits at a station on the ring, offset
  // along the ring's tangent. The rotated sector-search triple shares one
  // station (same datum); the chain triple sits at one station spread along
  // the tangent like pearls on a chain.
  struct Stop
  {
    NodeRole role;
    int station;
    float tangentOffsetM;
  };
  const Stop stops[] = {
    {NodeRole::PIN_SQUARE_SPIRAL_1, 0, 0.0f},
    {NodeRole::PIN_SQUARE_SPIRAL_2, 1, 0.0f},
    {NodeRole::PIN_LAWNMOWER_1, 2, 0.0f},
    {NodeRole::PIN_LAWNMOWER_2, 3, 0.0f},
    {NodeRole::PIN_GRID_PATTERN_1, 4, 0.0f},
    {NodeRole::PIN_VS_SEARCH_1, 5, 0.0f},
    {NodeRole::PIN_VS_SEARCH_2, 5, 0.0f},
    {NodeRole::PIN_VS_SEARCH_3, 5, 0.0f},
    {NodeRole::PIN_VS_CHAIN_1, 6, -kDemoChainSpacingM},
    {NodeRole::PIN_VS_CHAIN_2, 6, 0.0f},
    {NodeRole::PIN_VS_CHAIN_3, 6, kDemoChainSpacingM},
  };
  int stationCount = 0;
  for (const Stop & s : stops)
  {
    stationCount = std::max(stationCount, s.station + 1);
  }

  const float sign = kDemoRingClockwise ? 1.0f : -1.0f;
  double arc = 0.0;
  int pinIndex = 1;
  for (const Stop & s : stops)
  {
    const float angle =
      kDemoRingStartBearing + sign * 2.0f * static_cast<float>(M_PI) * s.station / stationCount;
    // tangent in the visiting direction
    const float tx = -sign * std::sin(angle);
    const float ty = sign * std::cos(angle);
    MissionNode pin;
    pin.name = "P" + std::to_string(pinIndex++);
    pin.role = s.role;
    pin.x = kDemoRingRadiusM * std::cos(angle) + s.tangentOffsetM * tx;
    pin.y = kDemoRingRadiusM * std::sin(angle) + s.tangentOffsetM * ty;
    pin.geo = geoOf(pin.x, pin.y);
    const MissionNode & prev = plan.nodes.back();
    arc += std::hypot(pin.x - prev.x, pin.y - prev.y);
    pin.arcLengthM = arc;
    plan.nodes.push_back(pin);
  }

  // tail: straight legs back to the centre of square spiral 1, south past
  // the spiral, then the figure-eight / loiter centroid
  auto addTailNode = [&plan, &arc, &geoOf](
                       const std::string & name, NodeRole role, float x, float y) {
    MissionNode n;
    n.name = name;
    n.role = role;
    n.x = x;
    n.y = y;
    n.geo = geoOf(x, y);
    n.approach = LegKind::GOTO;
    const MissionNode & prev = plan.nodes.back();
    arc += std::hypot(x - prev.x, y - prev.y);
    n.arcLengthM = arc;
    plan.nodes.push_back(n);
  };
  const MissionNode & p1 = plan.nodes.at(1);
  addTailNode("SquareCentre", NodeRole::WAYPOINT, p1.x, p1.y);

  const float southX = -kSouthWaypointSouthM;
  const float southY = -kSouthWaypointWestM;
  addTailNode("SouthWaypoint", NodeRole::WAYPOINT, southX, southY);

  const float feX = southX - kFigureEightOffsetSouthM;
  const float feY = southY;
  addTailNode("FigureEight", NodeRole::FIGURE_EIGHT_1, feX, feY);
  addTailNode("FigureEight", NodeRole::FIGURE_EIGHT_2, feX, feY);
  addTailNode("Centroid", NodeRole::LOITER_CENTROID, feX, feY);

  MissionNode landing;
  landing.role = NodeRole::LANDING_ZONE;
  landing.approach = LegKind::GOTO;
  bool hotel = false;
  if (landingSite == LandingSite::HOTEL)
  {
    if (backbone.size() >= 2 && project(backbone.back().lat, backbone.back().lon, landing.x, landing.y))
    {
      landing.name = backboneName(backbone.size() - 1);
      landing.geo = backbone.back();
      hotel = true;
    }
    else
    {
      RCLCPP_WARN(
        logger, "[planner] HOTEL landing requested but no projected backbone - landing at the island");
    }
  }
  if (!hotel)
  {
    landing.name = "Island";
    landing.geo = geoOf(0.0f, 0.0f);
  }
  {
    const MissionNode & prev = plan.nodes.back();
    landing.arcLengthM = arc + std::hypot(landing.x - prev.x, landing.y - prev.y);
  }
  plan.nodes.push_back(landing);
  plan.landingNodeIndex = static_cast<int>(plan.nodes.size()) - 1;

  detail::emitLegs(plan);
  plan.valid = !plan.legs.empty();

  RCLCPP_INFO(
    logger, "[planner] demo ring: r=%.0f m, %d stations, %d pins, landing at %s, %zu nodes, %zu legs",
    static_cast<double>(kDemoRingRadiusM), stationCount, kNumPinRoles, landing.name.c_str(),
    plan.nodes.size(), plan.legs.size());
  return plan;
}

// Isolated test: one leg centred on the takeoff point, then land at home.
inline MissionPlan buildTestLegPlan(
  const std::string & testLeg, float altitudeM, rclcpp::Logger logger)
{
  MissionPlan plan;
  plan.altitudeNedZ = -altitudeM;
  plan.testMode = true;
  plan.testLeg = testLeg;
  plan.source = "test_leg";

  MissionNode home;
  home.name = "Home";
  home.role = NodeRole::TAKEOFF;
  plan.nodes.push_back(home);

  auto addLanding = [&plan]() {
    MissionNode lz;
    lz.name = "Home";
    lz.role = NodeRole::LANDING_ZONE;
    plan.nodes.push_back(lz);
    plan.landingNodeIndex = static_cast<int>(plan.nodes.size()) - 1;
  };

  if (auto role = roleForSuperstateName(testLeg))
  {
    MissionNode pin;
    pin.name = testLeg;
    pin.role = *role;
    plan.nodes.push_back(pin);
    addLanding();

    MissionLeg pattern;
    pattern.index = 0;
    pattern.kind = LegKind::PATTERN;
    pattern.role = *role;
    pattern.fromNode = 1;
    pattern.toNode = 1;
    pattern.label = "@Home (test leg)";
    plan.legs.push_back(pattern);

    MissionLeg landing;
    landing.index = 1;
    landing.kind = LegKind::LANDING;
    landing.fromNode = 1;
    landing.toNode = 2;
    landing.label = "Home";
    plan.legs.push_back(landing);
  }
  else if (testLeg == "StGoToWaypoint")
  {
    MissionNode target;
    target.name = "TestTarget";
    target.role = NodeRole::WAYPOINT;
    target.x = kTestLegSyntheticDistanceM;
    target.approach = LegKind::GOTO;
    plan.nodes.push_back(target);
    addLanding();

    MissionLeg go;
    go.index = 0;
    go.kind = LegKind::GOTO;
    go.fromNode = 0;
    go.toNode = 1;
    go.lengthM = kTestLegSyntheticDistanceM;
    go.label = "Home -> TestTarget";
    plan.legs.push_back(go);

    MissionLeg landing;
    landing.index = 1;
    landing.kind = LegKind::LANDING;
    landing.fromNode = 1;
    landing.toNode = 2;
    landing.lengthM = kTestLegSyntheticDistanceM;
    landing.label = "TestTarget -> Home";
    plan.legs.push_back(landing);
  }
  else if (testLeg == "StSineWaveVertical" || testLeg == "StSineWaveHorizontal")
  {
    MissionNode target;
    target.name = "TestTarget";
    target.role = NodeRole::BACKBONE;
    target.x = kTestLegSyntheticDistanceM;
    target.y = 0.0f;
    plan.nodes.push_back(target);
    addLanding();

    MissionLeg transit;
    transit.index = 0;
    transit.kind = testLeg == "StSineWaveVertical" ? LegKind::TRANSIT_V : LegKind::TRANSIT_H;
    transit.fromNode = 0;
    transit.toNode = 1;
    transit.lengthM = kTestLegSyntheticDistanceM;
    transit.label = "Home -> TestTarget";
    plan.legs.push_back(transit);

    MissionLeg landing;
    landing.index = 1;
    landing.kind = LegKind::LANDING;
    landing.fromNode = 1;
    landing.toNode = 2;
    landing.lengthM = kTestLegSyntheticDistanceM;
    landing.label = "TestTarget -> Home";
    plan.legs.push_back(landing);
  }
  else
  {
    if (testLeg != "StGoToLandingZone")
    {
      RCLCPP_ERROR(
        logger, "[planner] unknown test_leg '%s' - flying landing-only plan", testLeg.c_str());
    }
    addLanding();
    MissionLeg landing;
    landing.index = 0;
    landing.kind = LegKind::LANDING;
    landing.fromNode = 0;
    landing.toNode = 1;
    landing.label = "Home";
    plan.legs.push_back(landing);
  }

  plan.valid = true;
  return plan;
}

}  // namespace railway
}  // namespace sm_cl_px4_mr_test_4
