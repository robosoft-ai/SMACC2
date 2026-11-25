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

#include <cl_rgb/box_edge_search.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace cl_rgb
{
namespace
{
// corners 0 to 3 on the table, 4 to 7 above them; the top face, then the four sides
constexpr int kFace[5][4] = {{4, 5, 6, 7}, {0, 1, 5, 4}, {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};

struct Drawn
{
  std::array<cv::Point2d, 8> pixel;
  std::array<bool, 5> faceSeen{};
  bool inImage = true;
};

Drawn draw(const PinholeCamera & camera, double tableZ, double x, double y, double yaw, double side)
{
  Drawn drawn;
  std::array<cv::Vec3d, 8> corner;
  const double h = side / 2.0, c = std::cos(yaw), s = std::sin(yaw);
  const double dx[4] = {-h, h, h, -h}, dy[4] = {-h, -h, h, h};
  for (int i = 0; i < 8; ++i)
  {
    corner[i] = cv::Vec3d(
      x + c * dx[i % 4] - s * dy[i % 4], y + s * dx[i % 4] + c * dy[i % 4], tableZ + (i < 4 ? 0.0 : side));
    const auto pixel = camera.project(corner[i]);
    drawn.inImage = drawn.inImage && pixel.has_value();
    drawn.pixel[i] = pixel.value_or(cv::Point2d(0.0, 0.0));
  }
  const cv::Vec3d middle = (corner[0] + corner[6]) * 0.5;
  for (int f = 0; f < 5; ++f)
  {
    const cv::Vec3d faceMiddle = (corner[kFace[f][0]] + corner[kFace[f][2]]) * 0.5;
    // a face is seen when the camera is on its outer side
    drawn.faceSeen[f] = (faceMiddle - middle).dot(camera.position - faceMiddle) > 0.0;
  }
  return drawn;
}

struct Gradients
{
  cv::Mat x, y, magnitude;  // the strongest of the three colour channels, per pixel
  cv::Mat edges;            // 255 on an edge
  cv::Mat lab;              // 32 bit float, L 0 to 100
};

struct EdgeCount
{
  double fraction = 0.0;
  double hidden = 1.0;
  double weakest = 0.0;  // the fraction of the edge that is least there, among those seen
};

EdgeCount countEdges(
  const Drawn & drawn, const Gradients & image, const cv::Mat & hidden, const BoxEdgeSearchParams & params)
{
  bool used[8][8] = {};
  int hit = 0, compared = 0, total = 0;
  double weakest = 1.0;
  bool anyEdge = false;
  for (int f = 0; f < 5; ++f)
  {
    if (!drawn.faceSeen[f])
    {
      continue;
    }
    for (int e = 0; e < 4; ++e)
    {
      const int a = std::min(kFace[f][e], kFace[f][(e + 1) % 4]);
      const int b = std::max(kFace[f][e], kFace[f][(e + 1) % 4]);
      if (used[a][b])
      {
        continue;
      }
      used[a][b] = true;
      const cv::Point2d along = drawn.pixel[b] - drawn.pixel[a];
      const double length = cv::norm(along);
      if (length < 1.0)
      {
        continue;
      }
      const cv::Point2d across(-along.y / length, along.x / length);
      const int steps = std::max(2, static_cast<int>(length / 1.5));
      int edgeHit = 0, edgeCompared = 0;
      for (int i = 0; i <= steps; ++i)
      {
        const double t = (1.0 - params.edgeMiddle) / 2.0 + params.edgeMiddle * i / steps;
        const cv::Point2d on = drawn.pixel[a] + along * t;
        ++total;
        const cv::Point at(static_cast<int>(on.x + 0.5), static_cast<int>(on.y + 0.5));
        const cv::Rect frame(0, 0, image.magnitude.cols, image.magnitude.rows);
        if (!frame.contains(at) || (!hidden.empty() && hidden.at<uchar>(at) != 0))
        {
          continue;
        }
        ++compared;
        ++edgeCompared;
        for (int offset = -1; offset <= 1; ++offset)
        {
          const cv::Point2d beside = on + across * static_cast<double>(offset);
          const cv::Point p(static_cast<int>(beside.x + 0.5), static_cast<int>(beside.y + 0.5));
          if (!frame.contains(p))
          {
            continue;
          }
          const float magnitude = image.magnitude.at<float>(p);
          const double acrossEdge =
            std::fabs(image.x.at<float>(p) * across.x + image.y.at<float>(p) * across.y);
          if (magnitude >= params.minGradient && acrossEdge > 0.85 * magnitude)
          {
            ++hit;
            ++edgeHit;
            break;
          }
        }
      }
      if (edgeCompared >= 3)
      {
        anyEdge = true;
        weakest = std::min(weakest, static_cast<double>(edgeHit) / edgeCompared);
      }
    }
  }
  EdgeCount count;
  if (total > 0 && compared > 0)
  {
    count.hidden = 1.0 - static_cast<double>(compared) / total;
    count.fraction = static_cast<double>(hit) / compared;
    count.weakest = anyEdge ? weakest : 0.0;
  }
  return count;
}

// density of image edges inside the seen faces, a few pixels away from the box's own edges
double innerEdges(
  const Drawn & drawn, const Gradients & image, const cv::Mat & hidden, const BoxEdgeSearchParams & params)
{
  cv::Rect window(
    static_cast<int>(drawn.pixel[0].x) - 40, static_cast<int>(drawn.pixel[0].y) - 50, 80, 80);
  window &= cv::Rect(0, 0, image.edges.cols, image.edges.rows);
  if (window.area() == 0)
  {
    return 1.0;
  }
  const cv::Mat shrink = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  double inner = 0.0;
  int area = 0;
  for (int f = 0; f < 5; ++f)
  {
    if (!drawn.faceSeen[f])
    {
      continue;
    }
    std::vector<cv::Point> corners;
    for (int i = 0; i < 4; ++i)
    {
      const cv::Point2d p = drawn.pixel[kFace[f][i]] - cv::Point2d(window.x, window.y);
      corners.emplace_back(static_cast<int>(p.x + 0.5), static_cast<int>(p.y + 0.5));
    }
    cv::Mat face = cv::Mat::zeros(window.size(), CV_8U);
    cv::fillConvexPoly(face, corners, 255);
    cv::erode(face, face, shrink);
    if (!hidden.empty())
    {
      face.setTo(0, hidden(window));
    }
    area += cv::countNonZero(face);
    // The edges inside this face: only the strong ones, and how far they all run one way. The
    // grain of wood is faint and runs one way; print and a table's pattern are neither.
    int count = 0;
    double sumCos = 0.0, sumSin = 0.0, sumSquare = 0.0;
    for (int v = 0; v < face.rows; ++v)
    {
      for (int u = 0; u < face.cols; ++u)
      {
        if (face.at<uchar>(v, u) == 0 || image.edges(window).at<uchar>(v, u) == 0)
        {
          continue;
        }
        const cv::Point at(u + window.x, v + window.y);
        const double gx = image.x.at<float>(at), gy = image.y.at<float>(at);
        const double square = gx * gx + gy * gy;
        if (std::sqrt(square) < params.innerMinGradient)
        {
          continue;
        }
        ++count;
        // direction doubled, so that the two senses of one direction add up
        sumCos += gx * gx - gy * gy;
        sumSin += 2.0 * gx * gy;
        sumSquare += square;
      }
    }
    const double oneWay = sumSquare > 0.0 ? std::hypot(sumCos, sumSin) / sumSquare : 0.0;  // 0 to 1
    inner += count * (1.0 - params.oneWayDiscount * oneWay);
  }
  return area >= 6 ? inner / area : 1.0;
}

// the largest colour difference between one of the seen faces and the ring just outside the box
double contrast(const Drawn & drawn, const Gradients & image, const cv::Mat & hidden)
{
  cv::Rect window(
    static_cast<int>(drawn.pixel[0].x) - 45, static_cast<int>(drawn.pixel[0].y) - 55, 90, 90);
  window &= cv::Rect(0, 0, image.lab.cols, image.lab.rows);
  if (window.area() == 0)
  {
    return 0.0;
  }
  const cv::Mat shrink = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  cv::Mat whole = cv::Mat::zeros(window.size(), CV_8U);
  std::vector<cv::Mat> faces;
  for (int f = 0; f < 5; ++f)
  {
    if (!drawn.faceSeen[f])
    {
      continue;
    }
    std::vector<cv::Point> corners;
    for (int i = 0; i < 4; ++i)
    {
      const cv::Point2d p = drawn.pixel[kFace[f][i]] - cv::Point2d(window.x, window.y);
      corners.emplace_back(static_cast<int>(p.x + 0.5), static_cast<int>(p.y + 0.5));
    }
    cv::Mat face = cv::Mat::zeros(window.size(), CV_8U);
    cv::fillConvexPoly(face, corners, 255);
    whole |= face;
    cv::erode(face, face, shrink);
    if (!hidden.empty())
    {
      face.setTo(0, hidden(window));
    }
    if (cv::countNonZero(face) >= 6)
    {
      faces.push_back(face);
    }
  }
  // the ring: a few pixels clear of the outline, so that neither the box's edge nor a pattern's
  // line along it is counted
  cv::Mat near, far;
  cv::dilate(whole, near, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)));
  cv::dilate(whole, far, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(19, 19)));
  cv::Mat ring = far - near;
  if (!hidden.empty())
  {
    ring.setTo(0, hidden(window));
  }
  if (cv::countNonZero(ring) < 12 || faces.empty())
  {
    return 0.0;
  }
  // the ring's middle colour, not its mean: a neighbouring object in part of it does not decide
  std::vector<float> channel[3];
  for (int v = 0; v < ring.rows; ++v)
  {
    for (int u = 0; u < ring.cols; ++u)
    {
      if (ring.at<uchar>(v, u) != 0)
      {
        const auto & pixel = image.lab(window).at<cv::Vec3f>(v, u);
        for (int c = 0; c < 3; ++c)
        {
          channel[c].push_back(pixel[c]);
        }
      }
    }
  }
  cv::Vec3d around;
  for (int c = 0; c < 3; ++c)
  {
    std::nth_element(channel[c].begin(), channel[c].begin() + channel[c].size() / 2, channel[c].end());
    around[c] = channel[c][channel[c].size() / 2];
  }
  double largest = 0.0;
  for (const auto & face : faces)
  {
    const cv::Scalar mean = cv::mean(image.lab(window), face);
    const double dl = 0.5 * (mean[0] - around[0]), da = mean[1] - around[1], db = mean[2] - around[2];
    largest = std::max(largest, std::sqrt(dl * dl + da * da + db * db));
  }
  return largest;
}
}  // namespace

cv::Mat colourEdges(const cv::Mat & bgr, double cannyLow, double cannyHigh)
{
  cv::Mat smooth;
  cv::GaussianBlur(bgr, smooth, cv::Size(3, 3), 0.0);
  std::vector<cv::Mat> channels;
  cv::split(smooth, channels);
  cv::Mat all = cv::Mat::zeros(bgr.size(), CV_8U);
  for (const auto & channel : channels)
  {
    cv::Mat edges;
    cv::Canny(channel, edges, cannyLow, cannyHigh);
    all |= edges;
  }
  return all;
}

double clutterAt(
  const cv::Mat & edges, const cv::Mat & hidden, const PinholeCamera & camera, double tableZ, double x,
  double y, double radiusM, double heightM)
{
  std::vector<cv::Point> rim;
  for (const double z : {tableZ, tableZ + heightM})
  {
    for (int i = 0; i < 16; ++i)
    {
      const double angle = i * M_PI / 8.0;
      const auto pixel =
        camera.project(cv::Vec3d(x + radiusM * std::cos(angle), y + radiusM * std::sin(angle), z));
      if (!pixel)
      {
        return 1.0;
      }
      rim.emplace_back(static_cast<int>(pixel->x + 0.5), static_cast<int>(pixel->y + 0.5));
    }
  }
  std::vector<cv::Point> hull;
  cv::convexHull(rim, hull);
  cv::Mat covered = cv::Mat::zeros(edges.size(), CV_8U);
  cv::fillConvexPoly(covered, hull, 255);
  const int area = cv::countNonZero(covered);
  if (area == 0)
  {
    return 1.0;
  }
  cv::Mat busy = edges & covered;
  if (!hidden.empty())
  {
    busy |= hidden & covered;
  }
  return static_cast<double>(cv::countNonZero(busy)) / area;
}

std::vector<BoxEdgeMatch> findBoxesByEdges(
  const cv::Mat & bgr, const PinholeCamera & camera, double tableZ, const cv::Mat & hidden,
  const BoxEdgeSearchParams & params)
{
  Gradients image;
  cv::Mat smooth;
  cv::GaussianBlur(bgr, smooth, cv::Size(3, 3), 0.0);
  std::vector<cv::Mat> channels;
  cv::split(smooth, channels);
  cv::Mat unit;
  smooth.convertTo(unit, CV_32F, 1.0 / 255.0);
  cv::cvtColor(unit, image.lab, cv::COLOR_BGR2Lab);
  image.x = cv::Mat::zeros(bgr.size(), CV_32F);
  image.y = image.x.clone();
  image.magnitude = image.x.clone();
  image.edges = cv::Mat::zeros(bgr.size(), CV_8U);
  for (const auto & channel : channels)
  {
    cv::Mat gx, gy, magnitude, edges;
    cv::Sobel(channel, gx, CV_32F, 1, 0, 3, 0.25);
    cv::Sobel(channel, gy, CV_32F, 0, 1, 3, 0.25);
    cv::magnitude(gx, gy, magnitude);
    const cv::Mat stronger = magnitude > image.magnitude;
    gx.copyTo(image.x, stronger);
    gy.copyTo(image.y, stronger);
    magnitude.copyTo(image.magnitude, stronger);
    cv::Canny(channel, edges, params.cannyLow, params.cannyHigh);
    image.edges |= edges;
  }

  const auto score = [&](BoxEdgeMatch & match)
  {
    const Drawn drawn = draw(camera, tableZ, match.x, match.y, match.yawRad, match.sideM);
    const EdgeCount count = drawn.inImage ? countEdges(drawn, image, hidden, params) : EdgeCount();
    match.hidden = count.hidden;
    match.weakestEdge = count.weakest;
    match.edgeFraction = count.hidden <= params.maxHidden ? count.fraction : 0.0;
    match.score = match.edgeFraction - params.hiddenCost * match.hidden;
  };

  std::vector<BoxEdgeMatch> matches;
  const int turns = std::max(1, static_cast<int>(std::lround(M_PI / 2.0 / params.yawStepRad)));
  for (const double side : params.sidesM)
  {
    for (double x = params.xMin; x <= params.xMax; x += params.stepM)
    {
      for (double y = params.yMin; y <= params.yMax; y += params.stepM)
      {
        BoxEdgeMatch best;
        for (int turn = 0; turn < turns; ++turn)
        {
          BoxEdgeMatch match;
          match.x = x;
          match.y = y;
          match.yawRad = turn * params.yawStepRad;
          match.sideM = side;
          score(match);
          if (match.score > best.score)
          {
            best = match;
          }
        }
        if (best.score > params.minEdgeFraction)
        {
          matches.push_back(best);
        }
      }
    }
  }
  // each good place again, finely
  const double fineYaw = params.yawStepRad / 4.0;
  for (auto & match : matches)
  {
    BoxEdgeMatch best = match;
    for (int ix = -3; ix <= 3; ++ix)
    {
      for (int iy = -3; iy <= 3; ++iy)
      {
        for (int turn = -3; turn <= 3; ++turn)
        {
          BoxEdgeMatch fine = match;
          fine.x += ix * 0.001;
          fine.y += iy * 0.001;
          fine.yawRad += turn * fineYaw;
          score(fine);
          if (fine.score > best.score)
          {
            best = fine;
          }
        }
      }
    }
    match = best;
    const Drawn drawn = draw(camera, tableZ, match.x, match.y, match.yawRad, match.sideM);
    match.innerEdges = innerEdges(drawn, image, hidden, params);
    match.score -= params.innerEdgeCost * match.innerEdges;
    match.contrast = contrast(drawn, image, hidden);
    match.score += params.weakestEdgeBonus * match.weakestEdge;
    match.score += params.contrastBonus * std::min(match.contrast, params.contrastFull) / params.contrastFull;
  }
  if (params.minWeakestEdge > 0.0)
  {
    matches.erase(
      std::remove_if(
        matches.begin(), matches.end(),
        [&params](const BoxEdgeMatch & match) { return match.weakestEdge < params.minWeakestEdge; }),
      matches.end());
  }
  if (params.minContrast > 0.0)
  {
    matches.erase(
      std::remove_if(
        matches.begin(), matches.end(),
        [&params](const BoxEdgeMatch & match) { return match.contrast < params.minContrast; }),
      matches.end());
  }
  std::sort(
    matches.begin(), matches.end(),
    [](const BoxEdgeMatch & a, const BoxEdgeMatch & b) { return a.score > b.score; });
  std::vector<BoxEdgeMatch> apart;
  for (const auto & match : matches)
  {
    const bool taken = std::any_of(
      apart.begin(), apart.end(), [&](const BoxEdgeMatch & other)
      { return std::hypot(match.x - other.x, match.y - other.y) < params.apartM; });
    if (!taken)
    {
      apart.push_back(match);
      if (static_cast<int>(apart.size()) >= params.maxMatches)
      {
        break;
      }
    }
  }
  return apart;
}

}  // namespace cl_rgb
