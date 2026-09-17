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

#include <cmath>
#include <cstddef>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// Mission plan data model. The plan is built once (MsInFlight::onEntry) and
// walked by StRailway; it lives on MsInFlight so it survives every inner
// transition. Only StRailway advances the cursor.

namespace sm_cl_px4_mr_test_4
{
namespace railway
{

enum class NodeRole
{
  TAKEOFF,
  BACKBONE,  // interpolated / pass-through node
  WAYPOINT,  // plain go-to node (no pattern)
  PIN_SQUARE_SPIRAL_1,
  PIN_SQUARE_SPIRAL_2,
  PIN_LAWNMOWER_1,
  PIN_LAWNMOWER_2,
  PIN_GRID_PATTERN_1,
  PIN_VS_SEARCH_1,
  PIN_VS_SEARCH_2,
  PIN_VS_SEARCH_3,
  PIN_VS_CHAIN_1,
  PIN_VS_CHAIN_2,
  PIN_VS_CHAIN_3,
  // tail patterns (leaf states, co-located at the figure-eight centre)
  FIGURE_EIGHT_1,
  FIGURE_EIGHT_2,
  LOITER_CENTROID,
  LANDING_ZONE
};

inline const char * roleName(NodeRole role)
{
  switch (role)
  {
    case NodeRole::TAKEOFF:
      return "TAKEOFF";
    case NodeRole::BACKBONE:
      return "BACKBONE";
    case NodeRole::WAYPOINT:
      return "WAYPOINT";
    case NodeRole::PIN_SQUARE_SPIRAL_1:
      return "SsSquareSpiral1";
    case NodeRole::PIN_SQUARE_SPIRAL_2:
      return "SsSquareSpiral2";
    case NodeRole::PIN_LAWNMOWER_1:
      return "SsLawnmower1";
    case NodeRole::PIN_LAWNMOWER_2:
      return "SsLawnmower2";
    case NodeRole::PIN_GRID_PATTERN_1:
      return "SsGridPattern1";
    case NodeRole::PIN_VS_SEARCH_1:
      return "SsVSSearch1";
    case NodeRole::PIN_VS_SEARCH_2:
      return "SsVSSearch2";
    case NodeRole::PIN_VS_SEARCH_3:
      return "SsVSSearch3";
    case NodeRole::PIN_VS_CHAIN_1:
      return "SsVSChain1";
    case NodeRole::PIN_VS_CHAIN_2:
      return "SsVSChain2";
    case NodeRole::PIN_VS_CHAIN_3:
      return "SsVSChain3";
    case NodeRole::FIGURE_EIGHT_1:
      return "StFigureEight1";
    case NodeRole::FIGURE_EIGHT_2:
      return "StFigureEight2";
    case NodeRole::LOITER_CENTROID:
      return "StLoiterCentroid";
    case NodeRole::LANDING_ZONE:
      return "LANDING_ZONE";
  }
  return "?";
}

// any role that dispatches a pattern state at its node
inline bool isPinRole(NodeRole role)
{
  return role != NodeRole::TAKEOFF && role != NodeRole::BACKBONE && role != NodeRole::WAYPOINT &&
         role != NodeRole::LANDING_ZONE;
}

// The superstate visiting order (pin k, 1-based)
inline constexpr NodeRole kPinRoles[] = {
  NodeRole::PIN_SQUARE_SPIRAL_1, NodeRole::PIN_SQUARE_SPIRAL_2, NodeRole::PIN_LAWNMOWER_1,
  NodeRole::PIN_LAWNMOWER_2,     NodeRole::PIN_GRID_PATTERN_1,  NodeRole::PIN_VS_SEARCH_1,
  NodeRole::PIN_VS_SEARCH_2,     NodeRole::PIN_VS_SEARCH_3,     NodeRole::PIN_VS_CHAIN_1,
  NodeRole::PIN_VS_CHAIN_2,      NodeRole::PIN_VS_CHAIN_3};
inline constexpr int kNumPinRoles = static_cast<int>(sizeof(kPinRoles) / sizeof(kPinRoles[0]));

// tail pattern roles (flown after the ring, before landing)
inline constexpr NodeRole kTailRoles[] = {
  NodeRole::FIGURE_EIGHT_1, NodeRole::FIGURE_EIGHT_2, NodeRole::LOITER_CENTROID};

// Pin k (1-based, in visiting order) -> superstate role
inline NodeRole pinRole(int k)
{
  if (k < 1) return kPinRoles[0];
  if (k > kNumPinRoles) return kPinRoles[kNumPinRoles - 1];
  return kPinRoles[k - 1];
}

// superstate name (as given to test_leg) -> role
inline std::optional<NodeRole> roleForSuperstateName(const std::string & name)
{
  for (NodeRole r : kPinRoles)
  {
    if (name == roleName(r))
    {
      return r;
    }
  }
  for (NodeRole r : kTailRoles)
  {
    if (name == roleName(r))
    {
      return r;
    }
  }
  return std::nullopt;
}

enum class LegKind
{
  TRANSIT_V,  // StSineWaveVertical
  TRANSIT_H,  // StSineWaveHorizontal
  GOTO,       // StGoToWaypoint (straight, CbGoToLocation)
  PATTERN,    // superstate at a pin (role says which)
  LANDING     // StGoToLandingZone -> StLoiterHotel -> MsLanding
};

inline const char * legKindName(LegKind kind)
{
  switch (kind)
  {
    case LegKind::TRANSIT_V:
      return "TRANSIT_V";
    case LegKind::TRANSIT_H:
      return "TRANSIT_H";
    case LegKind::GOTO:
      return "GOTO";
    case LegKind::PATTERN:
      return "PATTERN";
    case LegKind::LANDING:
      return "LANDING";
  }
  return "?";
}

struct MissionNode
{
  std::string name;
  cl_px4_mr::GeoPoint geo;
  float x = 0.0f;  // NED north (m) from the PX4 reference origin - the pattern CENTROID
  float y = 0.0f;  // NED east (m)
  // where the pattern flown at this node starts / ends (== x,y for plain nodes);
  // transit legs target the entry, leg lengths run from the previous exit
  float entryX = 0.0f;
  float entryY = 0.0f;
  float exitX = 0.0f;
  float exitY = 0.0f;
  double arcLengthM = 0.0;
  NodeRole role = NodeRole::BACKBONE;
  std::optional<LegKind> approach;  // how to reach this node; empty = alternating sine transit
};

struct MissionLeg
{
  int index = 0;
  LegKind kind = LegKind::TRANSIT_V;
  NodeRole role = NodeRole::BACKBONE;  // PATTERN legs: which superstate
  int fromNode = 0;
  int toNode = 0;
  float lengthM = 0.0f;
  std::string label;
};

struct MissionPlan
{
  std::vector<MissionNode> nodes;
  std::vector<MissionLeg> legs;
  size_t cursor = 0;           // leg to dispatch next (or being flown)
  bool legInProgress = false;  // set by StRailway when it dispatches; cleared on re-entry
  float altitudeNedZ = 0.0f;   // -kMissionAltitudeM
  int landingNodeIndex = 0;
  bool valid = false;
  bool testMode = false;
  std::string testLeg;
  std::string source;  // where the backbone came from

  bool exhausted() const { return cursor >= legs.size(); }
  const MissionLeg & current() const { return legs.at(cursor); }
  const MissionNode & node(int i) const { return nodes.at(static_cast<size_t>(i)); }
  const MissionNode & currentTargetNode() const { return node(current().toNode); }
  const MissionNode & landingNode() const { return node(landingNodeIndex); }

  std::string toTable() const
  {
    std::ostringstream os;
    os << "\n=== Mission plan (" << (testMode ? "TEST LEG '" + testLeg + "'" : "full mission")
       << ", source: " << source << ", valid: " << (valid ? "yes" : "NO")
       << ", altitude NED z=" << altitudeNedZ << ") ===\n";
    os << "Nodes:\n";
    for (size_t i = 0; i < nodes.size(); ++i)
    {
      const auto & n = nodes[i];
      char buf[256];
      std::snprintf(
        buf, sizeof(buf), "  N%-2zu %-18s %-18s lat=%.6f lon=%.6f  NED=(%9.1f, %9.1f)  s=%8.0f m", i,
        n.name.c_str(), roleName(n.role), n.geo.lat, n.geo.lon, static_cast<double>(n.x),
        static_cast<double>(n.y), n.arcLengthM);
      os << buf;
      if (std::fabs(n.entryX - n.x) > 0.5f || std::fabs(n.entryY - n.y) > 0.5f)
      {
        std::snprintf(
          buf, sizeof(buf), "  entry=(%+.0f, %+.0f)", static_cast<double>(n.entryX - n.x),
          static_cast<double>(n.entryY - n.y));
        os << buf;
      }
      os << "\n";
    }
    os << "Legs:\n";
    for (const auto & l : legs)
    {
      char buf[256];
      std::snprintf(
        buf, sizeof(buf), "  L%-2d %-10s %-18s N%d -> N%d  %8.0f m  %s\n", l.index,
        legKindName(l.kind), l.kind == LegKind::PATTERN ? roleName(l.role) : "", l.fromNode,
        l.toNode, static_cast<double>(l.lengthM), l.label.c_str());
      os << buf;
    }
    return os.str();
  }
};

}  // namespace railway
}  // namespace sm_cl_px4_mr_test_4
