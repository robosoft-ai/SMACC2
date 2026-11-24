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
#include <cl_rgb/table_survey.hpp>

#include <opencv2/core.hpp>

#include <optional>

namespace cl_rgb
{

// A box standing on the table: its centre on the table plane and how it is turned.
struct BoxFit
{
  double x = 0.0;  // m, world frame
  double y = 0.0;
  double yawRad = 0.0;   // of the box's x side about the vertical, in [0, pi/2) for a square box
  double overlap = 0.0;  // of the box's outline with the blob, 0 to 1 (intersection over union)
  bool topOnly = false;  // fitted to the top face alone: the sides were not part of the blob
};

struct BoxFitParams
{
  cv::Vec3d sizeM{0.035, 0.035, 0.035};  // along the box's x, y and the vertical
  double searchM = 0.012;                // the centre is looked for this far around the first guess
  double stepM = 0.002;
  double yawStepRad = 0.0436;  // 2.5 degrees
  double minOverlap = 0.6;     // below this there is no fit
  // A side face that looks like the table (or its shadow) is missing from the blob. When the
  // whole outline fits worse than this, the top face alone is tried as well and the better kept.
  double tryTopBelow = 0.85;
  // For a box of one strong colour: blob pixels that are darker than the blob's middle
  // lightness and have less than this fraction of its strong chroma (the 90th percentile) are
  // left out of the fit. They are shadow, which near a coloured box takes on some of its colour.
  // A pale but bright face of the box is kept. 0 uses the whole blob.
  double minChromaFraction = 0.0;
};

// Finds the place and the turn of a box of known size whose outline, as the camera would see
// it, covers the blob best. The first guess of the centre comes from the blob's centroid.
std::optional<BoxFit> fitBoxOnTable(
  const Blob & blob, const PinholeCamera & camera, double tableZ, const BoxFitParams & params);

}  // namespace cl_rgb
