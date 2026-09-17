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

#include <mutex>
#include <smacc2/smacc.hpp>
#include <string>
#include <vector>

#include <cl_px4_mr/utils/geo_utils.hpp>

namespace cl_px4_mr
{

struct KmlLoadResult
{
  bool ok = false;
  std::string error;
  size_t pointCount = 0;
};

// Holds the mission backbone: an ordered list of WGS84 waypoints read from a
// KML file. Scope (this build): the FIRST LineString found in the document, by
// geometry type - Placemark names, Folders and counts are ignored. Polygons,
// altitude semantics and the general KML parse contract are deferred.
//
// The loader is reference-agnostic: it stores lat/lon/alt only. Projection to
// the local NED frame belongs to the consumer (CpVehicleLocalPosition::projectToNed).
class CpKmlMissionLoader : public smacc2::ISmaccComponent
{
public:
  CpKmlMissionLoader();
  virtual ~CpKmlMissionLoader();

  void onInitialize() override;

  // Parse the file; on success the parsed points replace the stored mission.
  // On failure the stored mission is left untouched.
  KmlLoadResult loadFile(const std::string & absolutePath);

  // Replace the stored mission directly (used for compiled-in fallbacks).
  void setMission(std::vector<GeoPoint> points, const std::string & source);

  bool hasMission() const;
  std::vector<GeoPoint> getMission() const;  // copy under mutex
  std::string getSource() const;             // file path, or "fallback:<label>"
  void clear();

  // Pure parser (testable without ROS). Returns the first LineString's
  // coordinates; on error returns empty and fills `error`.
  static std::vector<GeoPoint> parseKmlString(const std::string & xml, std::string & error);

private:
  mutable std::mutex mutex_;
  std::vector<GeoPoint> mission_;
  std::string source_;
};

}  // namespace cl_px4_mr
