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

#include <vector>

namespace cl_rgb
{

// A box standing on the table, found by its edges.
struct BoxEdgeMatch
{
  double x = 0.0;  // m, world frame
  double y = 0.0;
  double yawRad = 0.0;
  double sideM = 0.0;  // of the model that fitted; rounded edges make it smaller than the box
  // Fraction of the points on the box's visible edges that lie on an image edge running the
  // same way, less what the edges inside its faces and its hidden part cost. 1 at best.
  double score = 0.0;
  double edgeFraction = 0.0;
  double innerEdges = 0.0;  // density of image edges inside the faces, 0 to 1
  double hidden = 0.0;      // fraction of the edge points behind a fixture
  // The same fraction for the one seen edge that is least there. A box is closed on every side:
  // the end of something longer has a box's outline on three sides and none on the fourth.
  double weakestEdge = 0.0;
  // How unlike the table around it the box's most distinct face is, in Lab units (lightness at
  // half weight). A pattern on the table that has a box's outline is the table's own colour.
  double contrast = 0.0;
};

struct BoxEdgeSearchParams
{
  std::vector<double> sidesM{0.030, 0.035, 0.040};  // cubes of these sides are looked for
  // the part of the table that is searched
  double xMin = -0.5;
  double xMax = 0.5;
  double yMin = -0.5;
  double yMax = 0.5;
  double stepM = 0.004;        // first pass; the good places are then refined to 1 mm
  double yawStepRad = 0.1745;  // 10 degrees, refined to 2.5
  double minGradient = 6.0;    // of 255 per pixel: weaker image edges do not count
  double minEdgeFraction = 0.55;
  double edgeMiddle = 0.7;     // the middle part of each edge that is compared: corners are round
  double innerEdgeCost = 1.0;  // plain faces: image edges inside them count against the box
  // Two allowances for the grain of wood, which is faint and runs one way: inner edges weaker
  // than innerMinGradient (of 255 per pixel) are not counted, and the edges inside a face count
  // less the more they all run one way, by oneWayDiscount when they all do (0 to 1).
  double innerMinGradient = 0.0;
  double oneWayDiscount = 0.0;
  double hiddenCost = 0.3;
  double maxHidden = 0.4;  // more of the edges than this behind a fixture: not considered
  double apartM = 0.032;   // two matches are at least this far apart
  double cannyLow = 25.0;
  double cannyHigh = 60.0;
  // A match none of whose faces differs from its surroundings by this much is left out; 0
  // keeps all. For tables with a pattern of box-like outlines (a honeycomb).
  // A match one of whose seen edges is there for less than this fraction is left out; 0 keeps all.
  double minWeakestEdge = 0.0;
  double weakestEdgeBonus = 0.0;  // added to the score in proportion to the weakest edge
  double minContrast = 0.0;
  // A box that stands out from the table counts for more: this much is added to the score of a
  // match whose contrast reaches contrastFull, in proportion below it. 0 adds nothing.
  double contrastBonus = 0.0;
  double contrastFull = 20.0;
  int maxMatches = 12;
};

// The edges of an image, taken in each colour channel: 255 on an edge.
cv::Mat colourEdges(const cv::Mat & bgr, double cannyLow = 25.0, double cannyHigh = 60.0);

// How much stands at a place on the table, by the image alone: the fraction of edge pixels (and
// of hidden ones) among the pixels a cylinder of this radius and height at (x, y) would cover.
double clutterAt(
  const cv::Mat & edges, const cv::Mat & hidden, const PinholeCamera & camera, double tableZ,
  double x, double y, double radiusM, double heightM);

// Looks for boxes by drawing their visible edges at every place, turn and size and comparing
// them with the edges of the image: needs no picture of the empty table and no table colour.
// hidden: 255 where a fixture stands in front of the table (may be empty). Best match first.
std::vector<BoxEdgeMatch> findBoxesByEdges(
  const cv::Mat & bgr, const PinholeCamera & camera, double tableZ, const cv::Mat & hidden,
  const BoxEdgeSearchParams & params);

}  // namespace cl_rgb
