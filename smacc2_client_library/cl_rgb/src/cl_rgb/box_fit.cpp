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

#include <cl_rgb/box_fit.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace cl_rgb
{
namespace
{
// The outline of the box in the blob's own image window; empty when a corner is not seen.
cv::Mat outline(
  const PinholeCamera & camera, const cv::Rect & box, double x, double y, double yaw, double tableZ,
  const cv::Vec3d & size, bool topOnly)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<cv::Point> corners;
  for (int i = topOnly ? 4 : 0; i < 8; ++i)
  {
    const double dx = ((i & 1) != 0 ? 0.5 : -0.5) * size[0];
    const double dy = ((i & 2) != 0 ? 0.5 : -0.5) * size[1];
    const double dz = (i & 4) != 0 ? size[2] : 0.0;
    const auto pixel =
      camera.project(cv::Vec3d(x + c * dx - s * dy, y + s * dx + c * dy, tableZ + dz));
    if (!pixel)
    {
      return cv::Mat();
    }
    corners.emplace_back(cvRound(pixel->x) - box.x, cvRound(pixel->y) - box.y);
  }
  std::vector<cv::Point> hull;
  cv::convexHull(corners, hull);
  cv::Mat drawn = cv::Mat::zeros(box.size(), CV_8UC1);
  cv::fillConvexPoly(drawn, hull, cv::Scalar(255));
  return drawn;
}

double hullArea(
  const PinholeCamera & camera, double x, double y, double yaw, double tableZ,
  const cv::Vec3d & size, bool topOnly)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<cv::Point2f> corners;
  for (int i = topOnly ? 4 : 0; i < 8; ++i)
  {
    const double dx = ((i & 1) != 0 ? 0.5 : -0.5) * size[0];
    const double dy = ((i & 2) != 0 ? 0.5 : -0.5) * size[1];
    const double dz = (i & 4) != 0 ? size[2] : 0.0;
    const auto pixel =
      camera.project(cv::Vec3d(x + c * dx - s * dy, y + s * dx + c * dy, tableZ + dz));
    if (!pixel)
    {
      return 0.0;
    }
    corners.emplace_back(static_cast<float>(pixel->x), static_cast<float>(pixel->y));
  }
  std::vector<cv::Point2f> hull;
  cv::convexHull(corners, hull);
  return cv::contourArea(hull);
}
}  // namespace

namespace
{
// The blob without its weakly coloured pixels; the blob itself when that leaves too little.
Blob strongColourPart(const Blob & whole, double minChromaFraction)
{
  if (minChromaFraction <= 0.0 || whole.crop.empty())
  {
    return whole;
  }
  cv::Mat floatCrop, lab;
  whole.crop.convertTo(floatCrop, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(floatCrop, lab, cv::COLOR_BGR2Lab);
  std::vector<float> chromas, lightnesses;
  cv::Mat chroma(lab.size(), CV_32FC1, cv::Scalar(0));
  cv::Mat lightness(lab.size(), CV_32FC1, cv::Scalar(0));
  for (int r = 0; r < lab.rows; ++r)
  {
    for (int c = 0; c < lab.cols; ++c)
    {
      if (whole.mask.at<uchar>(r, c))
      {
        const cv::Vec3f v = lab.at<cv::Vec3f>(r, c);
        chroma.at<float>(r, c) = std::hypot(v[1], v[2]);
        chromas.push_back(chroma.at<float>(r, c));
        lightness.at<float>(r, c) = v[0];
        lightnesses.push_back(v[0]);
      }
    }
  }
  if (chromas.empty())
  {
    return whole;
  }
  std::sort(chromas.begin(), chromas.end());
  const float strong = chromas[static_cast<size_t>(0.9 * (chromas.size() - 1))];
  if (strong < 20.0F)
  {
    return whole;  // a white, gray or black box has no strong colour to tell it from its shadow
  }
  Blob part = whole;
  std::sort(lightnesses.begin(), lightnesses.end());
  const float middle = lightnesses[lightnesses.size() / 2];
  part.mask = whole.mask &
              ((chroma >= static_cast<float>(minChromaFraction) * strong) | (lightness >= middle));
  part.areaPx = cv::countNonZero(part.mask);
  if (part.areaPx < whole.areaPx / 4)
  {
    return whole;
  }
  const cv::Moments m = cv::moments(part.mask, true);
  part.centroidPx = cv::Point2d(whole.box.x + m.m10 / m.m00, whole.box.y + m.m01 / m.m00);
  return part;
}
}  // namespace

std::optional<BoxFit> fitBoxOnTable(
  const Blob & seen, const PinholeCamera & camera, double tableZ, const BoxFitParams & params)
{
  if (seen.mask.empty() || seen.areaPx <= 0)
  {
    return std::nullopt;
  }
  const Blob blob = strongColourPart(seen, params.minChromaFraction);
  // the centroid of the outline lies near the box's centre at half its height
  const auto guess =
    camera.pixelToPlane(blob.centroidPx.x, blob.centroidPx.y, tableZ + params.sizeM[2] / 2.0);
  if (!guess)
  {
    return std::nullopt;
  }
  const bool square = std::abs(params.sizeM[0] - params.sizeM[1]) < 1e-6;
  const double yawEnd = square ? CV_PI / 2.0 : CV_PI;
  const int steps = static_cast<int>(std::round(params.searchM / params.stepM));

  cv::Mat visible;
  double seenArea = blob.areaPx;
  if (blob.hidden.size() == blob.mask.size() && cv::countNonZero(blob.hidden) > 0)
  {
    cv::bitwise_not(blob.hidden, visible);
    seenArea = cv::countNonZero(blob.mask & visible);
  }

  BoxFit best;
  best.overlap = -1.0;
  for (const bool topOnly : {false, true})
  {
    if (topOnly && best.overlap >= params.tryTopBelow)
    {
      break;
    }
    // the centroid of the top face lies at the box's centre at its full height
    const auto centre =
      topOnly ? camera.pixelToPlane(blob.centroidPx.x, blob.centroidPx.y, tableZ + params.sizeM[2])
              : guess;
    if (!centre)
    {
      continue;
    }
    for (double yaw = 0.0; yaw < yawEnd - 1e-9; yaw += params.yawStepRad)
    {
      for (int ix = -steps; ix <= steps; ++ix)
      {
        for (int iy = -steps; iy <= steps; ++iy)
        {
          const double x = (*centre)[0] + ix * params.stepM;
          const double y = (*centre)[1] + iy * params.stepM;
          const cv::Mat drawn = outline(camera, blob.box, x, y, yaw, tableZ, params.sizeM, topOnly);
          if (drawn.empty())
          {
            continue;
          }
          // The outline's area in pixels as drawn; what of it lies outside the window is
          // added from the polygon's area, so that a box partly outside is not favoured.
          const double drawnPx = cv::countNonZero(drawn);
          const double outside =
            std::max(hullArea(camera, x, y, yaw, tableZ, params.sizeM, topOnly) - drawnPx, 0.0);
          double whole = drawnPx + outside;
          double both = 0.0;
          if (visible.empty())
          {
            both = cv::countNonZero(drawn & blob.mask);
          }
          else
          {
            // what a fixture hides counts neither for nor against the box
            both = cv::countNonZero(drawn & blob.mask & visible);
            whole = cv::countNonZero(drawn & visible) + outside;
          }
          whole = std::max(whole, 1.0);
          const double overlap = both / (whole + seenArea - both);
          if (overlap > best.overlap)
          {
            best = BoxFit{x, y, yaw, overlap, topOnly, whole, seenArea};
          }
        }
      }
    }
  }
  if (best.overlap < params.minOverlap)
  {
    return std::nullopt;
  }
  return best;
}

}  // namespace cl_rgb
