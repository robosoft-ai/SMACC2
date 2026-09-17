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

#include <cl_px4_mr/utils/pattern_generators.hpp>

#include <algorithm>

namespace cl_px4_mr
{

using pattern_detail::altitudeToZ;
using pattern_detail::kNaN;
using pattern_detail::pick;

std::vector<NedPoint> generateFlightPatternAscend(const FlightPatternAscendParams & p, const NedPoint & current)
{
  NedPoint target;
  target.x = pick(p.targetX, current.x);
  target.y = pick(p.targetY, current.y);
  target.z = -p.altitudeAgl;

  const bool atXy = std::hypot(target.x - current.x, target.y - current.y) <= p.tolerance;
  if (atXy && std::fabs(current.z - target.z) <= p.tolerance)
  {
    return {target};
  }
  NedPoint start = current;
  start.yaw = kNaN;
  return {start, target};
}

std::vector<NedPoint> generateFlightPatternLoiter(const FlightPatternLoiterParams & p, const NedPoint & current)
{
  const float cx = pick(p.centerX, current.x);
  const float cy = pick(p.centerY, current.y);
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float r = std::max(p.radius, 0.5f);
  const int n = std::max(p.pointsPerCircle, 8);
  const float sign = turnSign(p.direction);

  if (p.count <= 0)
  {
    // nothing to fly: hold the entry point (the follower posts success at once)
    NedPoint here = current;
    here.z = z;
    here.yaw = kNaN;
    return {here};
  }
  const int count = p.count;

  // enter at the point of the circle nearest the vehicle; if at the centre
  // use the entry heading to pick a start
  const float dx = current.x - cx;
  const float dy = current.y - cy;
  float theta0 = std::hypot(dx, dy) > 0.1f ? std::atan2(dy, dx)
                                           : (std::isnan(current.yaw) ? 0.0f : current.yaw);

  std::vector<NedPoint> path;
  path.reserve(static_cast<size_t>(count * n + 1));
  for (int k = 0; k <= count * n; ++k)
  {
    const float theta = theta0 + sign * 2.0f * static_cast<float>(M_PI) * k / n;
    NedPoint v;
    v.x = cx + r * std::cos(theta);
    v.y = cy + r * std::sin(theta);
    v.z = z;
    v.yaw = p.faceCenter ? std::atan2(cy - v.y, cx - v.x) : kNaN;
    path.push_back(v);
  }
  return path;
}

namespace
{

struct Leg
{
  float ux = 0.0f, uy = 0.0f;  // unit direction
  float length = 0.0f;
};

Leg legFrom(const NedPoint & start, float endX, float endY)
{
  Leg leg;
  const float dx = endX - start.x;
  const float dy = endY - start.y;
  leg.length = std::hypot(dx, dy);
  if (leg.length > 1e-3f)
  {
    leg.ux = dx / leg.length;
    leg.uy = dy / leg.length;
  }
  return leg;
}

}  // namespace

std::vector<NedPoint> generateFlightPatternSineWaveVertical(
  const FlightPatternSineWaveVerticalParams & p, const NedPoint & current)
{
  const float endX = pick(p.endX, current.x);
  const float endY = pick(p.endY, current.y);
  const float zBase = altitudeToZ(p.baseAltitudeAgl, current.z);
  const float lambda = std::max(p.wavelength, 0.1f);
  const float spacing = std::max(p.sampleSpacing, 0.1f);
  const Leg leg = legFrom(current, endX, endY);

  std::vector<NedPoint> path;
  if (leg.length <= 1e-3f)
  {
    path.push_back(NedPoint{endX, endY, zBase, kNaN});
    return path;
  }

  const int n = static_cast<int>(std::floor(leg.length / spacing));
  path.reserve(static_cast<size_t>(n + 2));
  for (int k = 0; k < n; ++k)
  {
    const float s = k * spacing;
    NedPoint v;
    v.x = current.x + s * leg.ux;
    v.y = current.y + s * leg.uy;
    v.z = zBase - p.amplitude * std::sin(2.0f * static_cast<float>(M_PI) * s / lambda);
    path.push_back(v);
  }
  path.push_back(NedPoint{endX, endY, zBase, kNaN});
  return path;
}

std::vector<NedPoint> generateFlightPatternSineWaveHorizontal(
  const FlightPatternSineWaveHorizontalParams & p, const NedPoint & current)
{
  const float endX = pick(p.endX, current.x);
  const float endY = pick(p.endY, current.y);
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float lambda = std::max(p.wavelength, 0.1f);
  const float spacing = std::max(p.sampleSpacing, 0.1f);
  const Leg leg = legFrom(current, endX, endY);

  std::vector<NedPoint> path;
  if (leg.length <= 1e-3f)
  {
    path.push_back(NedPoint{endX, endY, z, kNaN});
    return path;
  }

  // left-of-track normal in NED (x north, y east): rotate u by -90 deg
  const float nx = leg.uy;
  const float ny = -leg.ux;

  const int n = static_cast<int>(std::floor(leg.length / spacing));
  path.reserve(static_cast<size_t>(n + 2));
  for (int k = 0; k < n; ++k)
  {
    const float s = k * spacing;
    const float offset = p.amplitude * std::sin(2.0f * static_cast<float>(M_PI) * s / lambda);
    NedPoint v;
    v.x = current.x + s * leg.ux + offset * nx;
    v.y = current.y + s * leg.uy + offset * ny;
    v.z = z;
    path.push_back(v);
  }
  path.push_back(NedPoint{endX, endY, z, kNaN});
  return path;
}

std::vector<NedPoint> generateFlightPatternSquareSpiral(const FlightPatternSquareSpiralParams & p, const NedPoint & current)
{
  const float ox = pick(p.originX, current.x);
  const float oy = pick(p.originY, current.y);
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float spacing = std::max(p.spacing, 0.5f);
  const float sign = turnSign(p.direction);
  float heading = pick(p.initialHeading, std::isnan(current.yaw) ? 0.0f : current.yaw);

  std::vector<NedPoint> path;
  path.reserve(static_cast<size_t>(std::max(p.numLegs, 0) + 1));
  NedPoint v;
  v.x = ox;
  v.y = oy;
  v.z = z;
  path.push_back(v);

  for (int i = 0; i < p.numLegs; ++i)
  {
    const float len = spacing * static_cast<float>(i / 2 + 1);
    if (p.maxLegLength > 0.0f && len > p.maxLegLength)
    {
      break;
    }
    v.x += len * std::cos(heading);
    v.y += len * std::sin(heading);
    path.push_back(v);
    heading += sign * static_cast<float>(M_PI) / 2.0f;
  }
  return path;
}

float flightPatternSquareSpiralLength(const FlightPatternSquareSpiralParams & p)
{
  const float spacing = std::max(p.spacing, 0.5f);
  float total = 0.0f;
  for (int i = 0; i < p.numLegs; ++i)
  {
    const float len = spacing * static_cast<float>(i / 2 + 1);
    if (p.maxLegLength > 0.0f && len > p.maxLegLength)
    {
      break;
    }
    total += len;
  }
  return total;
}

std::vector<NedPoint> generateFlightPatternSpiral(const FlightPatternSpiralParams & p, const NedPoint & current)
{
  const float cx = pick(p.centerX, current.x);
  const float cy = pick(p.centerY, current.y);
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float spacing = std::max(p.spacing, 0.5f);
  const float a = spacing / (2.0f * static_cast<float>(M_PI));  // r = a * theta
  const float rStart = std::max(p.startRadius, 0.0f);
  const float rEnd = std::max(p.endRadius, rStart);
  const float sign = turnSign(p.direction);
  const float ds = std::max(p.sampleSpacing, 0.1f);

  // angular origin: bearing of the vehicle from the centre, else entry heading
  const float dx = current.x - cx;
  const float dy = current.y - cy;
  const float theta0 = std::hypot(dx, dy) > 0.1f ? std::atan2(dy, dx)
                                                 : (std::isnan(current.yaw) ? 0.0f : current.yaw);

  const float thetaStart = rStart / a;
  const float thetaEnd = rEnd / a;

  std::vector<NedPoint> path;
  float theta = thetaStart;
  while (theta < thetaEnd)
  {
    const float r = a * theta;
    NedPoint v;
    v.x = cx + r * std::cos(theta0 + sign * theta);
    v.y = cy + r * std::sin(theta0 + sign * theta);
    v.z = z;
    path.push_back(v);
    theta += ds / std::max(r, a);  // constant arc-length steps
  }
  NedPoint last;
  last.x = cx + rEnd * std::cos(theta0 + sign * thetaEnd);
  last.y = cy + rEnd * std::sin(theta0 + sign * thetaEnd);
  last.z = z;
  path.push_back(last);

  if (p.inward)
  {
    std::reverse(path.begin(), path.end());
  }
  return path;
}

float flightPatternSpiralLength(const FlightPatternSpiralParams & p)
{
  // area between the radii divided by the track spacing
  const float rStart = std::max(p.startRadius, 0.0f);
  const float rEnd = std::max(p.endRadius, rStart);
  const float spacing = std::max(p.spacing, 0.5f);
  return static_cast<float>(M_PI) * (rEnd * rEnd - rStart * rStart) / spacing;
}

int flightPatternLawnmowerLaneCount(const FlightPatternLawnmowerParams & p)
{
  const float spacing = std::max(p.laneSpacing, 0.5f);
  return std::max(1, static_cast<int>(std::floor(std::max(p.width, 0.0f) / spacing + 1e-3f)) + 1);
}

float flightPatternLawnmowerLength(const FlightPatternLawnmowerParams & p)
{
  const int lanes = flightPatternLawnmowerLaneCount(p);
  return lanes * std::max(p.laneLength, 0.0f) + (lanes - 1) * std::max(p.laneSpacing, 0.5f);
}

std::vector<NedPoint> generateFlightPatternLawnmower(const FlightPatternLawnmowerParams & p, const NedPoint & current)
{
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float heading = pick(p.laneHeading, std::isnan(current.yaw) ? 0.0f : current.yaw);
  const float ux = std::cos(heading);
  const float uy = std::sin(heading);
  // side normal: RIGHT of the heading in NED (x north, y east) is (-uy, ux)
  const float side = turnSign(p.firstTurn);
  const float nx = -uy * side;
  const float ny = ux * side;

  const int lanes = flightPatternLawnmowerLaneCount(p);
  const float spacing = std::max(p.laneSpacing, 0.5f);
  const float length = std::max(p.laneLength, 0.0f);
  const float coveredWidth = (lanes - 1) * spacing;

  float ox = pick(p.originX, current.x);
  float oy = pick(p.originY, current.y);
  if (p.originIsCenter)
  {
    ox -= 0.5f * length * ux + 0.5f * coveredWidth * nx;
    oy -= 0.5f * length * uy + 0.5f * coveredWidth * ny;
  }

  std::vector<NedPoint> path;
  path.reserve(static_cast<size_t>(2 * lanes));
  for (int i = 0; i < lanes; ++i)
  {
    NedPoint a;
    a.x = ox + i * spacing * nx;
    a.y = oy + i * spacing * ny;
    a.z = z;
    NedPoint b;
    b.x = a.x + length * ux;
    b.y = a.y + length * uy;
    b.z = z;
    if (i % 2 == 0)
    {
      path.push_back(a);
      path.push_back(b);
    }
    else
    {
      path.push_back(b);
      path.push_back(a);
    }
  }
  return path;
}

float flightPatternGridPatternLength(const FlightPatternGridPatternParams & p)
{
  FlightPatternLawnmowerParams second = p.base;
  second.laneLength = p.base.width;
  second.width = p.base.laneLength;
  return flightPatternLawnmowerLength(p.base) + (p.secondPass ? flightPatternLawnmowerLength(second) : 0.0f);
}

std::vector<NedPoint> generateFlightPatternGridPattern(const FlightPatternGridPatternParams & p, const NedPoint & current)
{
  FlightPatternLawnmowerParams first = p.base;
  first.originIsCenter = true;  // the grid is always centred on the origin
  first.originX = pick(p.base.originX, current.x);
  first.originY = pick(p.base.originY, current.y);
  first.laneHeading = pick(p.base.laneHeading, std::isnan(current.yaw) ? 0.0f : current.yaw);

  std::vector<NedPoint> path = generateFlightPatternLawnmower(first, current);
  if (!p.secondPass || path.empty())
  {
    return path;
  }

  // second pass: rotated 90 degrees over the same rectangle; try the four
  // start corners (heading +-90, step side left/right) and take the one that
  // starts nearest the end of pass 1
  const NedPoint & joint = path.back();
  std::vector<NedPoint> best;
  float bestDist = std::numeric_limits<float>::max();
  for (int variant = 0; variant < 4; ++variant)
  {
    FlightPatternLawnmowerParams second = first;
    second.laneLength = first.width;
    second.width = first.laneLength;
    second.laneHeading =
      first.laneHeading + ((variant & 1) ? -1.0f : 1.0f) * static_cast<float>(M_PI) / 2.0f;
    second.firstTurn = (variant & 2) ? Turn::LEFT : Turn::RIGHT;
    std::vector<NedPoint> candidate = generateFlightPatternLawnmower(second, current);
    if (candidate.empty())
    {
      continue;
    }
    const float d = nedDistance(joint, candidate.front());
    if (d < bestDist)
    {
      bestDist = d;
      best = std::move(candidate);
    }
  }
  path.insert(path.end(), best.begin(), best.end());
  return path;
}

float flightPatternVSSearchLength(const FlightPatternVSSearchParams & p)
{
  return 9.0f * std::max(p.radius, 1.0f) * static_cast<float>(std::max(p.cycles, 1));
}

std::vector<NedPoint> generateFlightPatternVSSearch(const FlightPatternVSSearchParams & p, const NedPoint & current)
{
  const float dx = pick(p.datumX, current.x);
  const float dy = pick(p.datumY, current.y);
  const float z = altitudeToZ(p.altitudeAgl, current.z);
  const float r = std::max(p.radius, 1.0f);
  const float sign = turnSign(p.direction);
  const float h0 = pick(p.initialHeading, std::isnan(current.yaw) ? 0.0f : current.yaw);
  const int cycles = std::max(p.cycles, 1);

  // leg headings in units of 120 degrees relative to the cycle's base heading:
  // legs 3->4 and 6->7 continue straight through the datum
  static const int kSteps[9] = {0, 1, 2, 2, 0, 1, 1, 2, 0};
  const float third = 2.0f * static_cast<float>(M_PI) / 3.0f;

  std::vector<NedPoint> path;
  path.reserve(static_cast<size_t>(9 * cycles + 1));
  NedPoint v;
  v.x = dx;
  v.y = dy;
  v.z = z;
  path.push_back(v);

  for (int c = 0; c < cycles; ++c)
  {
    const float base = h0 + sign * p.reorientation * static_cast<float>(c);
    for (int leg = 0; leg < 9; ++leg)
    {
      const float heading = base + sign * third * static_cast<float>(kSteps[leg]);
      v.x += r * std::cos(heading);
      v.y += r * std::sin(heading);
      path.push_back(v);
    }
    // snap back onto the datum to cancel accumulated float drift
    path.back().x = dx;
    path.back().y = dy;
  }
  return path;
}

}  // namespace cl_px4_mr
