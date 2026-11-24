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

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <cmath>

using cl_rgb::PinholeCamera;
using cl_rgb::TableParams;

namespace
{
PinholeCamera headCamera()
{
  PinholeCamera camera;
  camera.fx = camera.fy = 288.13254;
  camera.cx = 320.0;
  camera.cy = 240.0;
  const double c = std::cos(CV_PI / 6.0), s = std::sin(CV_PI / 6.0);
  camera.rotation = cv::Matx33d(1.0, 0.0, 0.0, 0.0, -c, s, 0.0, -s, -c);
  camera.position = cv::Vec3d(0.0, -0.41, 1.308);
  return camera;
}

TableParams table()
{
  TableParams t;
  t.height = 0.765;
  t.xMin = -0.7;
  t.xMax = 0.7;
  t.yMin = -0.6;
  t.yMax = 0.5;
  return t;
}

// A brown table seen by the camera, grey outside it, with flat patches painted on the table plane.
cv::Mat render(
  const PinholeCamera & camera, const std::vector<std::pair<cv::Rect2d, cv::Scalar>> & patches)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(90, 90, 90));
  const cv::Mat region = cl_rgb::tableRegionMask(camera, image.size(), table());
  image.setTo(cv::Scalar(30, 60, 120), region);
  for (const auto & [rect, colour] : patches)
  {
    std::vector<cv::Point> corners;
    for (const cv::Point2d corner :
         {rect.tl(), cv::Point2d(rect.x + rect.width, rect.y), rect.br(),
          cv::Point2d(rect.x, rect.y + rect.height)})
    {
      const auto pixel = camera.project(cv::Vec3d(corner.x, corner.y, 0.765));
      corners.emplace_back(cvRound(pixel->x), cvRound(pixel->y));
    }
    cv::fillConvexPoly(image, corners, colour);
  }
  return image;
}
}  // namespace

TEST(TableSurvey, TheRegionMaskCoversTheTableAndNothingElse)
{
  const PinholeCamera camera = headCamera();
  const cv::Mat mask = cl_rgb::tableRegionMask(camera, cv::Size(640, 480), table());
  const auto centre = camera.project(cv::Vec3d(0.0, 0.0, 0.765));
  EXPECT_EQ(mask.at<uchar>(cvRound(centre->y), cvRound(centre->x)), 255);
  EXPECT_EQ(mask.at<uchar>(5, 320), 0);   // the far wall, above the table's far edge
  EXPECT_EQ(mask.at<uchar>(120, 10), 0);  // left of the table's far corner
  EXPECT_EQ(
    mask.at<uchar>(470, 320), 255);  // the near part of the table, at the image's lower edge
}

TEST(TableSurvey, FindsPatchesAndPlacesThemOnTheTable)
{
  const PinholeCamera camera = headCamera();
  // a blue square of 5 cm and a yellow bar of 16 by 3 cm lying along x
  const cv::Mat image = render(
    camera, {{cv::Rect2d(0.10, 0.05, 0.05, 0.05), cv::Scalar(230, 40, 40)},
             {cv::Rect2d(-0.30, -0.10, 0.16, 0.03), cv::Scalar(40, 220, 230)}});
  const cv::Mat region = cl_rgb::tableRegionMask(camera, image.size(), table());
  const cv::Mat foreground = cl_rgb::segmentForeground(image, region, cl_rgb::SegmenterParams());
  const auto blobs = cl_rgb::findBlobs(image, foreground, camera, table(), cl_rgb::BlobParams());
  ASSERT_EQ(blobs.size(), 2u);

  const cl_rgb::Blob & bar = blobs[0].lengthM > blobs[1].lengthM ? blobs[0] : blobs[1];
  const cl_rgb::Blob & square = blobs[0].lengthM > blobs[1].lengthM ? blobs[1] : blobs[0];
  EXPECT_NEAR(square.tablePoint[0], 0.125, 0.004);
  EXPECT_NEAR(square.tablePoint[1], 0.075, 0.004);
  EXPECT_NEAR(square.lengthM, 0.05, 0.008);
  EXPECT_NEAR(square.widthM, 0.05, 0.008);
  EXPECT_GT(square.meanBgr[0], 200.0);  // blue

  EXPECT_NEAR(bar.tablePoint[0], -0.22, 0.004);
  EXPECT_NEAR(bar.tablePoint[1], -0.085, 0.004);
  EXPECT_NEAR(bar.lengthM, 0.16, 0.01);
  EXPECT_NEAR(bar.widthM, 0.03, 0.008);
  EXPECT_NEAR(bar.axisRad, 0.0, 0.05);             // along x
  EXPECT_LT(bar.nearPoint[1], bar.tablePoint[1]);  // the near edge is on the camera's side
}

TEST(TableSurvey, AReferenceImageIgnoresWhatTheEmptySceneAlreadyShows)
{
  const PinholeCamera camera = headCamera();
  // The empty scene has a large white fixture on the table; the scene adds one patch.
  const std::pair<cv::Rect2d, cv::Scalar> fixture{
    cv::Rect2d(-0.6, 0.1, 0.5, 0.3), cv::Scalar(240, 240, 240)};
  const std::pair<cv::Rect2d, cv::Scalar> patch{
    cv::Rect2d(0.20, -0.20, 0.06, 0.06), cv::Scalar(200, 200, 60)};
  const cv::Mat empty = render(camera, {fixture});
  const cv::Mat scene = render(camera, {fixture, patch});
  const cv::Mat region = cl_rgb::tableRegionMask(camera, scene.size(), table());

  const cv::Mat foreground =
    cl_rgb::segmentAgainstReference(scene, empty, region, cl_rgb::SegmenterParams());
  const auto blobs = cl_rgb::findBlobs(scene, foreground, camera, table(), cl_rgb::BlobParams());
  ASSERT_EQ(blobs.size(), 1u);  // the patch, and not the fixture
  EXPECT_NEAR(blobs[0].tablePoint[0], 0.23, 0.004);
  EXPECT_NEAR(blobs[0].tablePoint[1], -0.17, 0.004);
}

TEST(TableSurvey, AnEmptyTableHasNoBlobs)
{
  const PinholeCamera camera = headCamera();
  const cv::Mat image = render(camera, {});
  const cv::Mat region = cl_rgb::tableRegionMask(camera, image.size(), table());
  const cv::Mat foreground = cl_rgb::segmentForeground(image, region, cl_rgb::SegmenterParams());
  EXPECT_TRUE(cl_rgb::findBlobs(image, foreground, camera, table(), cl_rgb::BlobParams()).empty());
}

// Two cubes that touch in the image came out as one blob in the benchmark (a yellow and a red).
TEST(TableSurvey, TouchingPatchesOfDifferentColourAreCutApart)
{
  const PinholeCamera camera = headCamera();
  const cv::Mat image = render(
    camera, {{cv::Rect2d(-0.30, -0.10, 0.04, 0.04), cv::Scalar(40, 220, 230)},   // yellow
             {cv::Rect2d(-0.26, -0.12, 0.04, 0.04), cv::Scalar(40, 40, 230)}});  // red, touching it
  const cv::Mat region = cl_rgb::tableRegionMask(camera, image.size(), table());
  const cv::Mat foreground = cl_rgb::segmentForeground(image, region, cl_rgb::SegmenterParams());
  cl_rgb::BlobParams params;
  EXPECT_EQ(cl_rgb::findBlobs(image, foreground, camera, table(), params).size(), 1u);
  params.colourEdgeHueDeg = 20.0;
  const auto blobs = cl_rgb::findBlobs(image, foreground, camera, table(), params);
  ASSERT_EQ(blobs.size(), 2u);
  EXPECT_NEAR(static_cast<double>(blobs[0].areaPx) / blobs[1].areaPx, 1.0, 0.35);
}

// A cube's shadow on the table was part of its blob in the benchmark and moved its centre.
TEST(TableSurvey, AShadowIsNotAnObject)
{
  const PinholeCamera camera = headCamera();
  const cv::Mat empty = render(camera, {});
  const std::pair<cv::Rect2d, cv::Scalar> patch{
    cv::Rect2d(0.20, -0.20, 0.06, 0.06), cv::Scalar(200, 200, 60)};
  cv::Mat scene = render(camera, {patch});
  // beside the patch, the table dimmed to seven tenths
  const auto corner = camera.project(cv::Vec3d(0.27, -0.20, 0.765));
  const cv::Rect shadow(cvRound(corner->x), cvRound(corner->y) - 14, 14, 14);
  scene(shadow) = scene(shadow) * 0.7;
  const cv::Mat region = cl_rgb::tableRegionMask(camera, scene.size(), table());

  cl_rgb::SegmenterParams params;
  const cv::Mat with = cl_rgb::segmentAgainstReference(scene, empty, region, params);
  params.shadowDarkest = 0.45;
  const cv::Mat without = cl_rgb::segmentAgainstReference(scene, empty, region, params);
  EXPECT_GT(cv::countNonZero(with(shadow)), 100);
  EXPECT_EQ(cv::countNonZero(without(shadow)), 0);
  EXPECT_GT(cv::countNonZero(without), 100);  // the patch is still there
}
