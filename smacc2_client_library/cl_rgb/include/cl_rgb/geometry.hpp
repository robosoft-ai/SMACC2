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

#include <opencv2/core.hpp>

#include <optional>

namespace cl_rgb
{

// A pinhole camera at a known pose. The optical frame follows the ROS convention:
// z forward, x right, y down. Lengths in metres, image coordinates in pixels.
struct PinholeCamera
{
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  double cy = 0.0;
  cv::Matx33d rotation = cv::Matx33d::eye();  // optical frame to world frame
  cv::Vec3d position{0.0, 0.0, 0.0};          // of the optical centre, world frame

  // direction, in the world frame, of the ray through a pixel (not normalized)
  cv::Vec3d rayDirection(double u, double v) const;

  // pixel of a world point; nothing when the point is behind the camera
  std::optional<cv::Point2d> project(const cv::Vec3d & point) const;

  // where the ray through a pixel meets the horizontal plane at height z of the world
  // frame; nothing when the ray does not reach the plane in front of the camera
  std::optional<cv::Vec3d> pixelToPlane(double u, double v, double z) const;

  // maps homogeneous pixels (u, v, 1) to homogeneous plane coordinates (x, y, 1) at height z
  cv::Matx33d imageToPlaneHomography(double z) const;
};

// A horizontal table top in the world frame.
struct TableParams
{
  double height = 0.0;  // m, z of the table top
  double xMin = -0.5;   // m
  double xMax = 0.5;
  double yMin = -0.5;
  double yMax = 0.5;
};

}  // namespace cl_rgb
