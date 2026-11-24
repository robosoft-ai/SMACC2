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

#include <cl_rgb/geometry.hpp>

#include <opencv2/core.hpp>
#include <smacc2/smacc.hpp>

#include <optional>

namespace cl_rgb
{

// The table: a horizontal plane of known height and extent. With it, one pixel of a
// calibrated camera is one point in the world, and no depth image is needed.
class CpTablePlane : public smacc2::ISmaccComponent
{
public:
  explicit CpTablePlane(const TableParams & params);
  ~CpTablePlane() override;

  const TableParams & params() const { return params_; }

  // The world point seen at a pixel, taken to lie heightAboveTable above the table top. Zero
  // gives the point on the table; half an object's height gives the object's middle.
  std::optional<cv::Vec3d> pointUnderPixel(
    const PinholeCamera & camera, double u, double v, double heightAboveTable = 0.0) const;

  // the pixels of an image of the given size that see the table top, 255 inside
  cv::Mat regionMask(const PinholeCamera & camera, const cv::Size & size) const;

  // whether a point lies over the table top, inset by margin
  bool contains(double x, double y, double margin = 0.0) const;

private:
  TableParams params_;
};

}  // namespace cl_rgb
