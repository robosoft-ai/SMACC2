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

#include <cl_rgb/table_survey.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cl_rgb
{

namespace
{
// Median and robust spread (1.4826 * median absolute deviation) of one float channel under a mask.
void robustStats(const cv::Mat & channel, const cv::Mat & mask, double & median, double & spread)
{
  std::vector<float> values;
  values.reserve(static_cast<size_t>(cv::countNonZero(mask)));
  for (int r = 0; r < channel.rows; ++r)
  {
    const float * c = channel.ptr<float>(r);
    const uchar * m = mask.ptr<uchar>(r);
    for (int col = 0; col < channel.cols; ++col)
    {
      if (m[col])
      {
        values.push_back(c[col]);
      }
    }
  }
  if (values.empty())
  {
    median = 0.0;
    spread = 1.0;
    return;
  }
  const auto mid = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  median = *mid;
  for (auto & v : values)
  {
    v = std::abs(v - static_cast<float>(median));
  }
  std::nth_element(values.begin(), mid, values.end());
  spread = 1.4826 * *mid;
}
}  // namespace

cv::Mat tableRegionMask(
  const PinholeCamera & camera, const cv::Size & size, const TableParams & table)
{
  cv::Mat mask(size, CV_8UC1, cv::Scalar(0));
  const cv::Matx33d H = camera.imageToPlaneHomography(table.height);
  const double drop = table.height - camera.position[2];
  for (int v = 0; v < size.height; ++v)
  {
    uchar * row = mask.ptr<uchar>(v);
    for (int u = 0; u < size.width; ++u)
    {
      const cv::Vec3d p = H * cv::Vec3d(u, v, 1.0);
      // p[2] is the vertical component of the ray; the ray must travel towards the plane
      if (std::abs(p[2]) < 1e-12 || drop / p[2] <= 0.0)
      {
        continue;
      }
      const double x = p[0] / p[2];
      const double y = p[1] / p[2];
      if (x >= table.xMin && x <= table.xMax && y >= table.yMin && y <= table.yMax)
      {
        row[u] = 255;
      }
    }
  }
  return mask;
}

cv::Mat segmentForeground(
  const cv::Mat & bgr, const cv::Mat & tableMask, const SegmenterParams & params)
{
  cv::Mat floatImage, lab;
  bgr.convertTo(floatImage, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(floatImage, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);

  cv::Mat distanceSquared(bgr.size(), CV_32FC1, cv::Scalar(0.0f));
  const double weights[3] = {params.lightnessWeight, 1.0, 1.0};
  for (int c = 0; c < 3; ++c)
  {
    double median = 0.0, spread = 1.0;
    robustStats(channels[c], tableMask, median, spread);
    // A floor on the spread: a table of one flat colour must not turn noise into foreground.
    spread = std::max(spread, 1.0);
    cv::Mat normalized = (channels[c] - median) * (weights[c] / spread);
    distanceSquared += normalized.mul(normalized);
  }

  cv::Mat foreground = (distanceSquared > params.threshold * params.threshold);
  cv::bitwise_and(foreground, tableMask, foreground);
  if (params.openPx > 1)
  {
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.openPx, params.openPx)));
  }
  if (params.closePx > 1)
  {
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.closePx, params.closePx)));
  }
  return foreground;
}

cv::Mat segmentAgainstReference(
  const cv::Mat & bgr, const cv::Mat & referenceBgr, const cv::Mat & tableMask,
  const SegmenterParams & params, cv::Mat * shadow)
{
  if (shadow != nullptr)
  {
    *shadow = cv::Mat::zeros(bgr.size(), CV_8UC1);
  }
  cv::Mat image, reference, imageLab, referenceLab;
  bgr.convertTo(image, CV_32FC3, 1.0 / 255.0);
  referenceBgr.convertTo(reference, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(image, imageLab, cv::COLOR_BGR2Lab);
  cv::cvtColor(reference, referenceLab, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> difference;
  cv::split(imageLab - referenceLab, difference);
  difference[0] *= params.referenceLightnessWeight;
  cv::Mat distanceSquared = difference[0].mul(difference[0]) + difference[1].mul(difference[1]) +
                            difference[2].mul(difference[2]);

  cv::Mat foreground = (distanceSquared > params.referenceThreshold * params.referenceThreshold);
  cv::bitwise_and(foreground, tableMask, foreground);
  if (params.shadowDarkest > 0.0)
  {
    for (int r = 0; r < foreground.rows; ++r)
    {
      for (int c = 0; c < foreground.cols; ++c)
      {
        if (!foreground.at<uchar>(r, c))
        {
          continue;
        }
        const cv::Vec3f seen = imageLab.at<cv::Vec3f>(r, c);
        const cv::Vec3f empty = referenceLab.at<cv::Vec3f>(r, c);
        if (empty[0] < 1.0F || seen[0] >= empty[0] || seen[0] < params.shadowDarkest * empty[0])
        {
          continue;
        }
        const double emptyChroma = std::hypot(empty[1], empty[2]);
        // the reference's chroma, had it this pixel's lightness
        const double expected = emptyChroma * seen[0] / empty[0];
        const double chroma = std::hypot(seen[1], seen[2]);
        double turn = std::abs(std::atan2(seen[2], seen[1]) - std::atan2(empty[2], empty[1]));
        turn = turn > CV_PI ? 2.0 * CV_PI - turn : turn;
        if (
          turn * 180.0 / CV_PI <= params.shadowHueDeg &&
          chroma >= params.shadowChromaLow * expected &&
          chroma <= params.shadowChromaHigh * emptyChroma)
        {
          foreground.at<uchar>(r, c) = 0;
          if (shadow != nullptr)
          {
            shadow->at<uchar>(r, c) = 255;
          }
        }
      }
    }
  }
  if (params.openPx > 1)
  {
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.openPx, params.openPx)));
  }
  if (params.closePx > 1)
  {
    cv::morphologyEx(
      foreground, foreground, cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(params.closePx, params.closePx)));
  }
  return foreground;
}

cv::Mat referenceFixtures(
  const cv::Mat & referenceBgr, const cv::Mat & tableMask, double chromaFraction, double hueDeg)
{
  cv::Mat floatImage, lab;
  referenceBgr.convertTo(floatImage, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(floatImage, lab, cv::COLOR_BGR2Lab);
  std::vector<cv::Mat> channels;
  cv::split(lab, channels);
  double a = 0.0, b = 0.0, spread = 1.0;
  robustStats(channels[1], tableMask, a, spread);
  robustStats(channels[2], tableMask, b, spread);
  const double tableChroma = std::hypot(a, b);
  const double tableHue = std::atan2(b, a);

  cv::Mat fixtures = cv::Mat::zeros(referenceBgr.size(), CV_8UC1);
  for (int r = 0; r < lab.rows; ++r)
  {
    for (int c = 0; c < lab.cols; ++c)
    {
      if (!tableMask.at<uchar>(r, c))
      {
        continue;
      }
      const cv::Vec3f v = lab.at<cv::Vec3f>(r, c);
      double turn = std::abs(std::atan2(v[2], v[1]) - tableHue);
      turn = turn > CV_PI ? 2.0 * CV_PI - turn : turn;
      if (std::hypot(v[1], v[2]) < chromaFraction * tableChroma || turn * 180.0 / CV_PI > hueDeg)
      {
        fixtures.at<uchar>(r, c) = 255;
      }
    }
  }
  cv::morphologyEx(
    fixtures, fixtures, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  cv::dilate(fixtures, fixtures, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
  return fixtures;
}

std::vector<Blob> findBlobs(
  const cv::Mat & bgr, const cv::Mat & foreground, const PinholeCamera & camera,
  const TableParams & table, const BlobParams & params, const cv::Mat & hidden)
{
  cv::Mat floatImage, lab;
  bgr.convertTo(floatImage, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(floatImage, lab, cv::COLOR_BGR2Lab);

  cv::Mat regions = foreground;
  if (params.colourEdgeHueDeg > 0.0)
  {
    regions = foreground.clone();
    for (int r = 0; r < lab.rows; ++r)
    {
      for (int c = 0; c < lab.cols; ++c)
      {
        if (!foreground.at<uchar>(r, c))
        {
          continue;
        }
        const cv::Vec3f here = lab.at<cv::Vec3f>(r, c);
        // the pixel two to the right, and the pixel two below
        for (int k = 0; k < 2; ++k)
        {
          const int nr = k == 0 ? r : r + 2;
          const int nc = k == 0 ? c + 2 : c;
          if (nr >= lab.rows || nc >= lab.cols || !foreground.at<uchar>(nr, nc))
          {
            continue;
          }
          const cv::Vec3f there = lab.at<cv::Vec3f>(nr, nc);
          if (
            std::hypot(here[1], here[2]) < params.colourEdgeMinChroma ||
            std::hypot(there[1], there[2]) < params.colourEdgeMinChroma)
          {
            continue;
          }
          double turn = std::abs(std::atan2(there[2], there[1]) - std::atan2(here[2], here[1]));
          turn = turn > CV_PI ? 2.0 * CV_PI - turn : turn;
          if (turn * 180.0 / CV_PI > params.colourEdgeHueDeg)
          {
            // the pixel between the two is the seam
            regions.at<uchar>((r + nr) / 2, (c + nc) / 2) = 0;
          }
        }
      }
    }
  }

  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(
    regions, labels, stats, centroids, params.colourEdgeHueDeg > 0.0 ? 4 : 8);
  const cv::Matx33d H = camera.imageToPlaneHomography(table.height);

  std::vector<Blob> blobs;
  for (int i = 1; i < count; ++i)
  {
    const int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < params.minAreaPx)
    {
      continue;
    }
    Blob blob;
    blob.areaPx = area;
    blob.box = cv::Rect(
      stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
      stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
    blob.centroidPx = cv::Point2d(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
    blob.mask = (labels(blob.box) == i);
    blob.crop = bgr(blob.box).clone();
    if (hidden.size() == bgr.size())
    {
      blob.hidden = hidden(blob.box).clone();
      // A blob that lies mostly on a fixture is the fixture itself, moved or lit differently
      // than in the reference picture, and no object on the table.
      if (2 * cv::countNonZero(blob.mask & blob.hidden) > area)
      {
        continue;
      }
    }
    const cv::Scalar meanBgr = cv::mean(bgr(blob.box), blob.mask);
    const cv::Scalar meanLab = cv::mean(lab(blob.box), blob.mask);
    blob.meanBgr = cv::Vec3d(meanBgr[0], meanBgr[1], meanBgr[2]);
    blob.meanLab = cv::Vec3d(meanLab[0], meanLab[1], meanLab[2]);

    if (const auto point = camera.pixelToPlane(blob.centroidPx.x, blob.centroidPx.y, table.height))
    {
      blob.tablePoint = *point;
    }

    // The blob's pixels on the table plane: its footprint as this camera sees it.
    std::vector<cv::Point2f> footprint;
    footprint.reserve(static_cast<size_t>(area));
    double nearest = std::numeric_limits<double>::max();
    for (int r = 0; r < blob.box.height; ++r)
    {
      const uchar * m = blob.mask.ptr<uchar>(r);
      for (int c = 0; c < blob.box.width; ++c)
      {
        if (!m[c])
        {
          continue;
        }
        const cv::Vec3d p = H * cv::Vec3d(blob.box.x + c, blob.box.y + r, 1.0);
        if (std::abs(p[2]) < 1e-12)
        {
          continue;
        }
        const cv::Point2f onTable(static_cast<float>(p[0] / p[2]), static_cast<float>(p[1] / p[2]));
        footprint.push_back(onTable);
        const double dx = onTable.x - camera.position[0];
        const double dy = onTable.y - camera.position[1];
        const double distance = dx * dx + dy * dy;
        if (distance < nearest)
        {
          nearest = distance;
          blob.nearPoint = cv::Vec3d(onTable.x, onTable.y, table.height);
        }
      }
    }
    if (footprint.size() >= 3)
    {
      const cv::RotatedRect extent = cv::minAreaRect(footprint);
      const bool widthIsLong = extent.size.width >= extent.size.height;
      blob.lengthM = widthIsLong ? extent.size.width : extent.size.height;
      blob.widthM = widthIsLong ? extent.size.height : extent.size.width;
      double angle = extent.angle * CV_PI / 180.0 + (widthIsLong ? 0.0 : CV_PI / 2.0);
      while (angle >= CV_PI / 2.0) angle -= CV_PI;
      while (angle < -CV_PI / 2.0) angle += CV_PI;
      blob.axisRad = angle;
    }
    blobs.push_back(std::move(blob));
  }

  std::sort(
    blobs.begin(), blobs.end(), [](const Blob & a, const Blob & b) { return a.areaPx > b.areaPx; });
  if (static_cast<int>(blobs.size()) > params.maxBlobs)
  {
    blobs.resize(static_cast<size_t>(params.maxBlobs));
  }
  for (size_t i = 0; i < blobs.size(); ++i)
  {
    blobs[i].id = static_cast<int>(i);
  }
  return blobs;
}

}  // namespace cl_rgb
