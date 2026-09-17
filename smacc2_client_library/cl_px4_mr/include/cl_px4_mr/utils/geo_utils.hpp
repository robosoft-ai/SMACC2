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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

// Geodetic helpers for cl_px4_mr.
//
// MapProjection is a verbatim port of PX4's azimuthal equidistant projection
// (PX4-Autopilot/src/lib/geo/geo.cpp, spherical earth R = 6371000 m) so that
// lat/lon projected here land on exactly the same local NED coordinates the
// FMU uses for its own local position frame (whose origin is
// VehicleLocalPosition.ref_lat/ref_lon). Do not swap this for an ellipsoidal
// projection: the two disagree by ~0.3-0.5 % in scale, i.e. tens of metres at
// 10 km.

namespace cl_px4_mr
{

// WGS84 geographic point, degrees / metres
struct GeoPoint
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

// Local NED point (metres, z negative = up). yaw in radians, NaN = free.
struct NedPoint
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float yaw = std::numeric_limits<float>::quiet_NaN();
};

constexpr double kEarthRadiusM = 6371000.0;  // == PX4 CONSTANTS_RADIUS_OF_EARTH

namespace geo_detail
{
inline double radians(double deg) { return deg * (M_PI / 180.0); }
inline double degrees(double rad) { return rad * (180.0 / M_PI); }
inline double constrain(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
}  // namespace geo_detail

// Azimuthal equidistant projection about a reference lat/lon (PX4 MapProjection).
class MapProjection
{
public:
  MapProjection() = default;

  MapProjection(double lat0, double lon0, uint64_t timestamp = 0)
  {
    initReference(lat0, lon0, timestamp);
  }

  void initReference(double lat0, double lon0, uint64_t timestamp = 0)
  {
    refTimestamp_ = timestamp;
    refLat_ = geo_detail::radians(lat0);
    refLon_ = geo_detail::radians(lon0);
    refSinLat_ = std::sin(refLat_);
    refCosLat_ = std::cos(refLat_);
    initialized_ = true;
  }

  bool isInitialized() const { return initialized_; }
  uint64_t referenceTimestamp() const { return refTimestamp_; }
  double referenceLat() const { return geo_detail::degrees(refLat_); }
  double referenceLon() const { return geo_detail::degrees(refLon_); }

  // lat/lon in degrees -> x north, y east (metres)
  void project(double lat, double lon, float & x, float & y) const
  {
    const double latRad = geo_detail::radians(lat);
    const double lonRad = geo_detail::radians(lon);

    const double sinLat = std::sin(latRad);
    const double cosLat = std::cos(latRad);
    const double cosDLon = std::cos(lonRad - refLon_);

    const double arg =
      geo_detail::constrain(refSinLat_ * sinLat + refCosLat_ * cosLat * cosDLon, -1.0, 1.0);
    const double c = std::acos(arg);

    double k = 1.0;
    if (std::fabs(c) > 0.0)
    {
      k = c / std::sin(c);
    }

    x = static_cast<float>(
      k * (refCosLat_ * sinLat - refSinLat_ * cosLat * cosDLon) * kEarthRadiusM);
    y = static_cast<float>(k * cosLat * std::sin(lonRad - refLon_) * kEarthRadiusM);
  }

  // x north, y east (metres) -> lat/lon in degrees
  void reproject(float x, float y, double & lat, double & lon) const
  {
    const double xRad = static_cast<double>(x) / kEarthRadiusM;
    const double yRad = static_cast<double>(y) / kEarthRadiusM;
    const double c = std::sqrt(xRad * xRad + yRad * yRad);

    if (std::fabs(c) > 0.0)
    {
      const double sinC = std::sin(c);
      const double cosC = std::cos(c);

      const double latRad = std::asin(cosC * refSinLat_ + (xRad * sinC * refCosLat_) / c);
      const double lonRad =
        refLon_ + std::atan2(yRad * sinC, c * refCosLat_ * cosC - xRad * refSinLat_ * sinC);

      lat = geo_detail::degrees(latRad);
      lon = geo_detail::degrees(lonRad);
    }
    else
    {
      lat = geo_detail::degrees(refLat_);
      lon = geo_detail::degrees(refLon_);
    }
  }

private:
  uint64_t refTimestamp_ = 0;
  double refLat_ = 0.0;
  double refLon_ = 0.0;
  double refSinLat_ = 0.0;
  double refCosLat_ = 1.0;
  bool initialized_ = false;
};

// Great-circle distance in metres (haversine, spherical earth; same model as
// PX4 get_distance_to_next_waypoint).
inline double haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
  const double p1 = geo_detail::radians(lat1);
  const double p2 = geo_detail::radians(lat2);
  const double dLat = p2 - p1;
  const double dLon = geo_detail::radians(lon2 - lon1);

  const double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
                   std::sin(dLon / 2.0) * std::sin(dLon / 2.0) * std::cos(p1) * std::cos(p2);
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return kEarthRadiusM * c;
}

inline double haversineDistance(const GeoPoint & a, const GeoPoint & b)
{
  return haversineDistance(a.lat, a.lon, b.lat, b.lon);
}

// Initial bearing from point 1 to point 2, radians, NED convention
// (0 = north, +pi/2 = east), wrapped to [-pi, pi].
inline double initialBearing(double lat1, double lon1, double lat2, double lon2)
{
  const double p1 = geo_detail::radians(lat1);
  const double p2 = geo_detail::radians(lat2);
  const double dLon = geo_detail::radians(lon2 - lon1);

  const double y = std::sin(dLon) * std::cos(p2);
  const double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dLon);
  return std::atan2(y, x);
}

// Euclidean 3D distance between two NED points
inline float nedDistance(const NedPoint & a, const NedPoint & b)
{
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  const float dz = b.z - a.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline float nedDistanceXY(const NedPoint & a, const NedPoint & b)
{
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

// cumulative[i] = arc length from vertex 0 to vertex i (double accumulation,
// cast to float at the end so long paths do not drift)
inline std::vector<float> cumulativeLengths(const std::vector<NedPoint> & path)
{
  std::vector<float> cum;
  cum.reserve(path.size());
  double acc = 0.0;
  for (size_t i = 0; i < path.size(); ++i)
  {
    if (i > 0)
    {
      acc += static_cast<double>(nedDistance(path[i - 1], path[i]));
    }
    cum.push_back(static_cast<float>(acc));
  }
  return cum;
}

inline float polylineLength(const std::vector<NedPoint> & path)
{
  if (path.empty())
  {
    return 0.0f;
  }
  return cumulativeLengths(path).back();
}

// Point at arc length s along the polyline (linear interpolation, clamped to
// the ends). `cum` must be cumulativeLengths(path). Returns the segment index
// containing s through `segmentIndex` (index of the segment start vertex).
inline NedPoint sampleAtArcLength(
  const std::vector<NedPoint> & path, const std::vector<float> & cum, float s,
  size_t * segmentIndex = nullptr)
{
  if (path.empty())
  {
    return NedPoint{};
  }
  if (path.size() == 1 || s <= 0.0f)
  {
    if (segmentIndex) *segmentIndex = 0;
    return path.front();
  }
  if (s >= cum.back())
  {
    if (segmentIndex) *segmentIndex = path.size() - 2;
    return path.back();
  }

  // first vertex whose cumulative length exceeds s
  auto it = std::upper_bound(cum.begin(), cum.end(), s);
  size_t i1 = static_cast<size_t>(it - cum.begin());
  size_t i0 = i1 - 1;
  const float segLen = cum[i1] - cum[i0];
  const float t = segLen > 0.0f ? (s - cum[i0]) / segLen : 0.0f;

  if (segmentIndex) *segmentIndex = i0;

  NedPoint p;
  p.x = path[i0].x + t * (path[i1].x - path[i0].x);
  p.y = path[i0].y + t * (path[i1].y - path[i0].y);
  p.z = path[i0].z + t * (path[i1].z - path[i0].z);
  p.yaw = path[i0].yaw;  // yaw is per-vertex, not interpolated
  return p;
}

// Insert intermediate vertices so that no segment is longer than `spacing`.
// Original vertices are kept (yaw preserved); inserted vertices carry NaN yaw.
inline std::vector<NedPoint> resamplePolyline(const std::vector<NedPoint> & path, float spacing)
{
  std::vector<NedPoint> out;
  if (path.empty() || spacing <= 0.0f)
  {
    return path;
  }
  out.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i)
  {
    const NedPoint & a = path[i - 1];
    const NedPoint & b = path[i];
    const float len = nedDistance(a, b);
    const int n = std::max(1, static_cast<int>(std::ceil(len / spacing)));
    for (int k = 1; k < n; ++k)
    {
      const float t = static_cast<float>(k) / static_cast<float>(n);
      NedPoint p;
      p.x = a.x + t * (b.x - a.x);
      p.y = a.y + t * (b.y - a.y);
      p.z = a.z + t * (b.z - a.z);
      out.push_back(p);
    }
    out.push_back(b);
  }
  return out;
}

}  // namespace cl_px4_mr
