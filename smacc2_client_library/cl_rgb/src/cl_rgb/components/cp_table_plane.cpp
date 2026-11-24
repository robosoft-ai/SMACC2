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

#include <cl_rgb/components/cp_table_plane.hpp>
#include <cl_rgb/table_survey.hpp>

namespace cl_rgb
{

CpTablePlane::CpTablePlane(const TableParams & params) : params_(params) {}

CpTablePlane::~CpTablePlane() {}

std::optional<cv::Vec3d> CpTablePlane::pointUnderPixel(
  const PinholeCamera & camera, double u, double v, double heightAboveTable) const
{
  return camera.pixelToPlane(u, v, params_.height + heightAboveTable);
}

cv::Mat CpTablePlane::regionMask(const PinholeCamera & camera, const cv::Size & size) const
{
  return tableRegionMask(camera, size, params_);
}

bool CpTablePlane::contains(double x, double y, double margin) const
{
  return x >= params_.xMin + margin && x <= params_.xMax - margin && y >= params_.yMin + margin &&
         y <= params_.yMax - margin;
}

}  // namespace cl_rgb
