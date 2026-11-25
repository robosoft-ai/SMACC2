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

#include <cl_rgb/rim_edge_search.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace cl_rgb
{
namespace
{
struct Gradients
{
  cv::Mat x, y, magnitude, lab;
};

bool drawRim(
  const PinholeCamera & camera, double tableZ, const RimEdgeMatch & match, double height, int points,
  std::vector<cv::Point2d> & pixels)
{
  pixels.resize(static_cast<size_t>(points));
  for (int i = 0; i < points; ++i)
  {
    const double angle = 2.0 * M_PI * i / points;
    const auto pixel = camera.project(
      cv::Vec3d(match.x + match.radiusM * std::cos(angle), match.y + match.radiusM * std::sin(angle), tableZ + height));
    if (!pixel)
    {
      return false;
    }
    pixels[static_cast<size_t>(i)] = *pixel;
  }
  return true;
}

void scoreRim(
  RimEdgeMatch & match, double height, const PinholeCamera & camera, double tableZ, const Gradients & image,
  const cv::Mat & hidden, const RimEdgeSearchParams & params)
{
  std::vector<cv::Point2d> pixels;
  match.edgeFraction = 0.0;
  match.score = 0.0;
  if (!drawRim(camera, tableZ, match, height, params.points, pixels))
  {
    return;
  }
  const int n = params.points;
  const cv::Rect frame(0, 0, image.magnitude.cols, image.magnitude.rows);
  int hit = 0, seen = 0;
  int partHit[4] = {0, 0, 0, 0}, partSeen[4] = {0, 0, 0, 0};
  for (int i = 0; i < n; ++i)
  {
    const cv::Point2d along = pixels[static_cast<size_t>((i + 1) % n)] - pixels[static_cast<size_t>((i + n - 1) % n)];
    const double length = cv::norm(along);
    const cv::Point2d & on = pixels[static_cast<size_t>(i)];
    const cv::Point at(static_cast<int>(on.x + 0.5), static_cast<int>(on.y + 0.5));
    if (length < 1e-6 || !frame.contains(at) || (!hidden.empty() && hidden.at<uchar>(at) != 0))
    {
      continue;
    }
    ++seen;
    ++partSeen[i * 4 / n];
    const cv::Point2d across(-along.y / length, along.x / length);
    for (int offset = -1; offset <= 1; ++offset)
    {
      const cv::Point2d beside = on + across * static_cast<double>(offset);
      const cv::Point p(static_cast<int>(beside.x + 0.5), static_cast<int>(beside.y + 0.5));
      if (!frame.contains(p))
      {
        continue;
      }
      const float magnitude = image.magnitude.at<float>(p);
      if (
        magnitude >= params.minGradient &&
        std::fabs(image.x.at<float>(p) * across.x + image.y.at<float>(p) * across.y) > 0.85 * magnitude)
      {
        ++hit;
        ++partHit[i * 4 / n];
        break;
      }
    }
  }
  match.hidden = 1.0 - static_cast<double>(seen) / n;
  if (seen == 0 || match.hidden > params.maxHidden)
  {
    return;
  }
  match.edgeFraction = static_cast<double>(hit) / seen;
  match.weakestPart = 1.0;
  for (int q = 0; q < 4; ++q)
  {
    if (partSeen[q] >= 3)
    {
      match.weakestPart = std::min(match.weakestPart, static_cast<double>(partHit[q]) / partSeen[q]);
    }
  }
  match.score = match.edgeFraction - params.hiddenCost * match.hidden;
}

double rimContrast(
  const RimEdgeMatch & match, double height, const PinholeCamera & camera, double tableZ, const Gradients & image,
  const RimEdgeSearchParams & params)
{
  std::vector<cv::Point2d> pixels;
  if (!drawRim(camera, tableZ, match, height, params.points, pixels))
  {
    return 0.0;
  }
  std::vector<cv::Point> outline;
  for (const auto & pixel : pixels)
  {
    outline.emplace_back(static_cast<int>(pixel.x + 0.5), static_cast<int>(pixel.y + 0.5));
  }
  cv::Rect window = cv::boundingRect(outline);
  window.x -= 12;
  window.y -= 12;
  window.width += 24;
  window.height += 24;
  window &= cv::Rect(0, 0, image.lab.cols, image.lab.rows);
  if (window.area() == 0)
  {
    return 0.0;
  }
  for (auto & point : outline)
  {
    point -= window.tl();
  }
  cv::Mat inside = cv::Mat::zeros(window.size(), CV_8U);
  cv::fillConvexPoly(inside, outline, 255);
  cv::Mat inner, near, far;
  cv::erode(inside, inner, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));
  cv::dilate(inside, near, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
  cv::dilate(inside, far, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(21, 21)));
  const cv::Mat ring = far - near;
  if (cv::countNonZero(inner) < 10 || cv::countNonZero(ring) < 10)
  {
    return 0.0;
  }
  const cv::Scalar a = cv::mean(image.lab(window), inner), b = cv::mean(image.lab(window), ring);
  const double dl = 0.5 * (a[0] - b[0]), da = a[1] - b[1], db = a[2] - b[2];
  return std::sqrt(dl * dl + da * da + db * db);
}
}  // namespace

std::vector<RimEdgeMatch> findRimsByEdges(
  const cv::Mat & bgr, const PinholeCamera & camera, double tableZ, const cv::Mat & hidden,
  const RimEdgeSearchParams & params)
{
  Gradients image;
  cv::Mat smooth, unit;
  cv::GaussianBlur(bgr, smooth, cv::Size(3, 3), 0.0);
  smooth.convertTo(unit, CV_32F, 1.0 / 255.0);
  cv::cvtColor(unit, image.lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(smooth, channels);
  image.x = cv::Mat::zeros(bgr.size(), CV_32F);
  image.y = image.x.clone();
  image.magnitude = image.x.clone();
  for (const auto & channel : channels)
  {
    cv::Mat gx, gy, magnitude;
    cv::Sobel(channel, gx, CV_32F, 1, 0, 3, 0.25);
    cv::Sobel(channel, gy, CV_32F, 0, 1, 3, 0.25);
    cv::magnitude(gx, gy, magnitude);
    const cv::Mat stronger = magnitude > image.magnitude;
    gx.copyTo(image.x, stronger);
    gy.copyTo(image.y, stronger);
    magnitude.copyTo(image.magnitude, stronger);
  }

  std::vector<RimEdgeMatch> all;
  for (size_t s = 0; s < params.shapes.size(); ++s)
  {
    const RimShape & shape = params.shapes[s];
    std::vector<RimEdgeMatch> matches;
    for (double x = params.xMin; x <= params.xMax; x += params.stepM)
    {
      for (double y = params.yMin; y <= params.yMax; y += params.stepM)
      {
        RimEdgeMatch match;
        match.x = x;
        match.y = y;
        match.radiusM = shape.radiusM;
        match.shape = static_cast<int>(s);
        scoreRim(match, shape.heightM, camera, tableZ, image, hidden, params);
        if (match.score > params.minEdgeFraction)
        {
          matches.push_back(match);
        }
      }
    }
    for (auto & match : matches)
    {
      RimEdgeMatch best = match;
      for (int ix = -3; ix <= 3; ++ix)
      {
        for (int iy = -3; iy <= 3; ++iy)
        {
          for (double dr = -params.radiusSlackM; dr <= params.radiusSlackM + 1e-9; dr += 0.001)
          {
            RimEdgeMatch fine = match;
            fine.x += ix * 0.001;
            fine.y += iy * 0.001;
            fine.radiusM = shape.radiusM + dr;
            scoreRim(fine, shape.heightM, camera, tableZ, image, hidden, params);
            if (fine.score > best.score)
            {
              best = fine;
            }
          }
        }
      }
      match = best;
      match.contrast = rimContrast(match, shape.heightM, camera, tableZ, image, params);
      match.score += params.weakestPartBonus * match.weakestPart +
        params.contrastBonus * std::min(match.contrast, params.contrastFull) / params.contrastFull;
    }
    std::sort(
      matches.begin(), matches.end(),
      [](const RimEdgeMatch & a, const RimEdgeMatch & b) { return a.score > b.score; });
    std::vector<RimEdgeMatch> apart;
    for (const auto & match : matches)
    {
      const bool taken = std::any_of(
        apart.begin(), apart.end(), [&](const RimEdgeMatch & other)
        { return std::hypot(match.x - other.x, match.y - other.y) < shape.radiusM + other.radiusM - 0.05; });
      if (!taken && static_cast<int>(apart.size()) < params.maxMatchesPerShape)
      {
        apart.push_back(match);
      }
    }
    all.insert(all.end(), apart.begin(), apart.end());
  }
  std::stable_sort(
    all.begin(), all.end(), [](const RimEdgeMatch & a, const RimEdgeMatch & b) { return a.score > b.score; });
  return all;
}

std::vector<RimEdgeMatch> sameShapeRims(
  const std::vector<RimEdgeMatch> & matches, const std::vector<RimShape> & shapes, int count, double tie)
{
  std::vector<RimEdgeMatch> chosen;
  double best = -1.0;
  // shapes in order of radius, so that a later shape that fits about as well takes over
  std::vector<size_t> order(shapes.size());
  for (size_t i = 0; i < order.size(); ++i)
  {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return shapes[a].radiusM < shapes[b].radiusM; });
  for (const size_t s : order)
  {
    std::vector<RimEdgeMatch> ofShape;
    double sum = 0.0;
    for (const auto & match : matches)
    {
      if (match.shape == static_cast<int>(s) && static_cast<int>(ofShape.size()) < count)
      {
        ofShape.push_back(match);
        sum += match.score;
      }
    }
    if (static_cast<int>(ofShape.size()) == count && sum > best - tie)
    {
      chosen = ofShape;
      best = std::max(best, sum);
    }
  }
  return chosen;
}

}  // namespace cl_rgb
