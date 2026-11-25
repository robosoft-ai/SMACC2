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

#include <string>
#include <vector>

namespace cl_rgb
{

// One connected region of the image that does not look like table.
struct Blob
{
  int id = 0;
  cv::Rect box;  // pixels
  int areaPx = 0;
  cv::Point2d centroidPx;
  // Where the ray through the centroid meets the table. For an object with height this lies
  // behind the object as seen from the camera; see CpTablePlane::pointUnderPixel for the
  // correction when the height is known.
  cv::Vec3d tablePoint{0.0, 0.0, 0.0};
  // Where the blob's point nearest to the camera meets the table: the object's near edge.
  cv::Vec3d nearPoint{0.0, 0.0, 0.0};
  double axisRad = 0.0;  // direction of the long axis on the table, world frame, in [-pi/2, pi/2)
  double lengthM = 0.0;  // extent of the footprint along the long axis
  double widthM = 0.0;   // and across it
  cv::Vec3d meanLab{0.0, 0.0, 0.0};  // L 0 to 100, a and b around 0
  cv::Vec3d meanBgr{0.0, 0.0, 0.0};
  cv::Mat mask;  // box-sized, 255 on the blob
  // box-sized, 255 where something of the empty scene stands in front of the table (the robot,
  // a fixture): the object may go on behind it. Empty when that is not known.
  cv::Mat hidden;
  cv::Mat crop;  // box-sized copy of the image
};

struct SegmenterParams
{
  // without a reference image: distance from the table's own colour
  double threshold = 3.5;        // in robust standard deviations
  double lightnessWeight = 0.4;  // lightness counts this much against the two colour channels
  // with a reference image of the empty scene: distance from that image, pixel by pixel
  std::string referenceImagePath;    // empty: no reference
  double referenceThreshold = 14.0;  // in Lab units
  // Against a reference picture: a shadow is the table darker but of the table's own hue. A
  // pixel is shadow when its lightness is below the reference's but not below this fraction of
  // it, its hue is within shadowHueDeg of the reference's, and its chroma is at least
  // shadowChromaLow of what the reference would have at that lightness and at most
  // shadowChromaHigh times the reference's own (a deep shadow is more saturated for its
  // lightness than the lit table). A black object has no colour and a strongly coloured one
  // too much, so neither is taken for shadow. 0 keeps shadows.
  double shadowDarkest = 0.0;
  double shadowHueDeg = 15.0;
  double shadowChromaLow = 0.5;
  double shadowChromaHigh = 2.2;
  double referenceLightnessWeight = 1.3;  // shading changes lightness more than colour
  int openPx = 3;                         // removes specks smaller than this
  int closePx = 5;                        // closes gaps smaller than this
};

struct BlobParams
{
  int minAreaPx = 40;
  int maxBlobs = 64;  // the largest ones are kept
  // Objects of different colour that touch in the image are cut apart where the hue changes by
  // more than this over two pixels. Hue and not brightness: the lit and the shaded face of one
  // object stay together. 0 leaves touching objects joined.
  double colourEdgeHueDeg = 0.0;
  double colourEdgeMinChroma = 12.0;  // Lab units; below this a pixel has no hue to compare
};

// The pixels that see the table top, 255 inside.
cv::Mat tableRegionMask(
  const PinholeCamera & camera, const cv::Size & size, const TableParams & table);

// The pixels of the table region whose colour is far from the table's own. The table's colour
// is taken from the region itself (median and spread in Lab), so the table must fill most of it.
cv::Mat segmentForeground(
  const cv::Mat & bgr, const cv::Mat & tableMask, const SegmenterParams & params);

// The pixels of the table region that differ from a reference image of the empty scene, taken
// by the same camera from the same place. Whatever the reference shows besides the table (a
// fixture, the robot at rest) is no foreground as long as it has not moved.
// `shadow`, when given, receives the pixels taken for shadow: no foreground, but not known to be
// empty table either, since a dark object of the table's own hue looks the same.
cv::Mat segmentAgainstReference(
  const cv::Mat & bgr, const cv::Mat & referenceBgr, const cv::Mat & tableMask,
  const SegmenterParams & params, cv::Mat * shadow = nullptr);

// What stands in the empty scene in front of the table (a robot at rest, a fixture): the pixels
// of the table's region in the reference picture that are much duller than the table (chroma
// below chromaFraction of the table's middle chroma) or of another hue (more than hueDeg away).
// Brightness is left out, so the table's own shading is not a fixture. For a coloured table.
cv::Mat referenceFixtures(
  const cv::Mat & referenceBgr, const cv::Mat & tableMask, double chromaFraction = 0.45,
  double hueDeg = 40.0);

// Groups the foreground into blobs and places each on the table. With `hidden` (see
// referenceFixtures) each blob learns what may hide part of it, and a blob lying mostly on a
// fixture is left out.
std::vector<Blob> findBlobs(
  const cv::Mat & bgr, const cv::Mat & foreground, const PinholeCamera & camera,
  const TableParams & table, const BlobParams & params, const cv::Mat & hidden = cv::Mat());

}  // namespace cl_rgb
