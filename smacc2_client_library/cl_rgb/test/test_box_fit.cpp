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

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

using cl_rgb::Blob;
using cl_rgb::PinholeCamera;

namespace
{
const double kTableZ = 0.765;
const double kSide = 0.035;

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

// The blob a cube at (x, y), turned by yaw, leaves in the head camera's image.
Blob cubeBlob(const PinholeCamera & camera, double x, double y, double yaw, bool topOnly = false)
{
  std::vector<cv::Point> corners;
  for (int i = topOnly ? 4 : 0; i < 8; ++i)
  {
    const double dx = ((i & 1) != 0 ? 0.5 : -0.5) * kSide, dy = ((i & 2) != 0 ? 0.5 : -0.5) * kSide;
    const cv::Vec3d corner(
      x + std::cos(yaw) * dx - std::sin(yaw) * dy, y + std::sin(yaw) * dx + std::cos(yaw) * dy,
      kTableZ + ((i & 4) != 0 ? kSide : 0.0));
    const auto pixel = camera.project(corner);
    corners.emplace_back(cvRound(pixel->x), cvRound(pixel->y));
  }
  std::vector<cv::Point> hull;
  cv::convexHull(corners, hull);
  cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
  cv::fillConvexPoly(image, hull, cv::Scalar(255));
  Blob blob;
  blob.box = cv::boundingRect(hull);
  blob.mask = image(blob.box).clone();
  blob.areaPx = cv::countNonZero(blob.mask);
  const cv::Moments m = cv::moments(blob.mask, true);
  blob.centroidPx = cv::Point2d(blob.box.x + m.m10 / m.m00, blob.box.y + m.m01 / m.m00);
  return blob;
}
}  // namespace

TEST(BoxFit, FindsPlaceAndTurnOfACube)
{
  const PinholeCamera camera = headCamera();
  const double places[][3] = {
    {0.078, -0.133, 0.0}, {-0.349, -0.134, 0.35}, {0.226, -0.034, 0.79}, {-0.15, -0.20, 1.2}};
  for (const auto & place : places)
  {
    const Blob blob = cubeBlob(camera, place[0], place[1], place[2]);
    const auto fit = cl_rgb::fitBoxOnTable(blob, camera, kTableZ, cl_rgb::BoxFitParams());
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->x, place[0], 0.003);
    EXPECT_NEAR(fit->y, place[1], 0.003);
    double difference = std::fmod(std::abs(fit->yawRad - place[2]), CV_PI / 2.0);
    difference = std::min(difference, CV_PI / 2.0 - difference);
    EXPECT_LT(difference, 0.09) << "cube turned by " << place[2];
  }
}

TEST(BoxFit, NoFitForSomethingThatIsNotTheBox)
{
  const PinholeCamera camera = headCamera();
  Blob blob;
  blob.box = cv::Rect(300, 200, 90, 12);  // a long thin strip
  blob.mask = cv::Mat(blob.box.size(), CV_8UC1, cv::Scalar(255));
  blob.areaPx = 90 * 12;
  blob.centroidPx = cv::Point2d(345.0, 206.0);
  EXPECT_FALSE(cl_rgb::fitBoxOnTable(blob, camera, kTableZ, cl_rgb::BoxFitParams()).has_value());
}

// An orange cube's shaded sides look like the table in the benchmark: only its top is a blob.
TEST(BoxFit, FitsTheTopFaceWhenTheSidesAreNotSeen)
{
  const PinholeCamera camera = headCamera();
  const Blob blob = cubeBlob(camera, -0.097, -0.017, 0.5, true);
  const auto fit = cl_rgb::fitBoxOnTable(blob, camera, kTableZ, cl_rgb::BoxFitParams());
  ASSERT_TRUE(fit.has_value());
  EXPECT_TRUE(fit->topOnly);
  EXPECT_NEAR(fit->x, -0.097, 0.003);
  EXPECT_NEAR(fit->y, -0.017, 0.003);
  EXPECT_NEAR(fit->yawRad, 0.5, 0.09);
}

// In the benchmark a red cube's shadow, tinted by the cube, was part of its blob: the cube stood
// square to the table and was measured turned by 45 degrees.
TEST(BoxFit, AWeaklyColouredShadowIsLeftOutOfTheFit)
{
  const PinholeCamera camera = headCamera();
  Blob blob = cubeBlob(camera, -0.19, -0.035, 0.0);
  // widen the window and add a dull shadow to the lower right of the cube
  const cv::Rect box(blob.box.x, blob.box.y, blob.box.width + 14, blob.box.height + 12);
  cv::Mat mask = cv::Mat::zeros(box.size(), CV_8UC1);
  blob.mask.copyTo(mask(cv::Rect(0, 0, blob.box.width, blob.box.height)));
  cv::Mat crop(box.size(), CV_8UC3, cv::Scalar(40, 60, 110));  // the table
  crop.setTo(cv::Scalar(30, 30, 230), mask);                   // the cube, red
  const cv::Rect shadow(blob.box.width - 8, blob.box.height - 10, 20, 20);
  cv::Mat shadowMask = cv::Mat::zeros(box.size(), CV_8UC1);
  shadowMask(shadow & cv::Rect(0, 0, box.width, box.height)) = 255;
  shadowMask.setTo(0, mask);
  crop.setTo(cv::Scalar(18, 30, 75), shadowMask);  // dark, slightly red
  mask |= shadowMask;
  blob.box = box;
  blob.mask = mask;
  blob.crop = crop;
  blob.areaPx = cv::countNonZero(mask);
  const cv::Moments m = cv::moments(mask, true);
  blob.centroidPx = cv::Point2d(box.x + m.m10 / m.m00, box.y + m.m01 / m.m00);

  cl_rgb::BoxFitParams params;
  params.minChromaFraction = 0.6;
  const auto fit = cl_rgb::fitBoxOnTable(blob, camera, kTableZ, params);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->x, -0.19, 0.004);
  EXPECT_NEAR(fit->y, -0.035, 0.004);
  const double turn = std::min(fit->yawRad, CV_PI / 2.0 - fit->yawRad);
  EXPECT_LT(turn, 0.09);
}

// The benchmark's cyan cube has a pale, bright top and strongly coloured sides. Leaving out
// every weakly coloured pixel took its top away and moved its measured place by 3 cm.
TEST(BoxFit, APaleBrightFaceStaysInTheFit)
{
  const PinholeCamera camera = headCamera();
  Blob blob = cubeBlob(camera, -0.28, -0.03, 0.6);
  const Blob top = cubeBlob(camera, -0.28, -0.03, 0.6, true);
  cv::Mat crop(blob.box.size(), CV_8UC3, cv::Scalar(40, 60, 110));
  crop.setTo(cv::Scalar(210, 190, 40), blob.mask);  // the sides, strong cyan
  cv::Mat topMask = cv::Mat::zeros(blob.box.size(), CV_8UC1);
  top.mask.copyTo(topMask(
    cv::Rect(top.box.x - blob.box.x, top.box.y - blob.box.y, top.box.width, top.box.height)));
  crop.setTo(cv::Scalar(250, 250, 215), topMask & blob.mask);  // the top, nearly white
  blob.crop = crop;

  cl_rgb::BoxFitParams params;
  params.minChromaFraction = 0.6;
  const auto fit = cl_rgb::fitBoxOnTable(blob, camera, kTableZ, params);
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->x, -0.28, 0.004);
  EXPECT_NEAR(fit->y, -0.03, 0.004);
}
