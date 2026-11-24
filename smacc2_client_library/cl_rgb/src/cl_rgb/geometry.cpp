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

#include <cl_rgb/geometry.hpp>

namespace cl_rgb
{

cv::Vec3d PinholeCamera::rayDirection(double u, double v) const
{
  return rotation * cv::Vec3d((u - cx) / fx, (v - cy) / fy, 1.0);
}

std::optional<cv::Point2d> PinholeCamera::project(const cv::Vec3d & point) const
{
  const cv::Vec3d p = rotation.t() * (point - position);
  if (p[2] <= 1e-9)
  {
    return std::nullopt;
  }
  return cv::Point2d(fx * p[0] / p[2] + cx, fy * p[1] / p[2] + cy);
}

std::optional<cv::Vec3d> PinholeCamera::pixelToPlane(double u, double v, double z) const
{
  const cv::Vec3d d = rayDirection(u, v);
  if (std::abs(d[2]) < 1e-12)
  {
    return std::nullopt;
  }
  const double s = (z - position[2]) / d[2];
  if (s <= 0.0)
  {
    return std::nullopt;
  }
  return position + s * d;
}

cv::Matx33d PinholeCamera::imageToPlaneHomography(double z) const
{
  // With m = R K^-1 (u, v, 1), the plane point is x = ox + (z - oz) m1 / m3 and the same for y.
  const cv::Matx33d inverseK(1.0 / fx, 0.0, -cx / fx, 0.0, 1.0 / fy, -cy / fy, 0.0, 0.0, 1.0);
  const double h = z - position[2];
  const cv::Matx33d toPlane(h, 0.0, position[0], 0.0, h, position[1], 0.0, 0.0, 1.0);
  return toPlane * rotation * inverseK;
}

}  // namespace cl_rgb
