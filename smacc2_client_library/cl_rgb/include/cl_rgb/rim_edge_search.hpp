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

// The round rim of something standing on the table (a bowl, a cup, a plate): a horizontal circle
// of a radius at a height.
struct RimShape
{
  double radiusM = 0.06;
  double heightM = 0.05;
};

struct RimEdgeMatch
{
  double x = 0.0;  // m, world frame, the circle's centre
  double y = 0.0;
  double radiusM = 0.0;
  int shape = 0;  // which of the shapes looked for
  double score = 0.0;
  double edgeFraction = 0.0;  // of the rim's seen points that lie on an image edge running along it
  double weakestPart = 0.0;   // the same for the quarter of the rim that is least there
  double hidden = 0.0;        // fraction of the rim behind a fixture
  double contrast = 0.0;      // the inside of the rim against a ring outside it, Lab units
};

struct RimEdgeSearchParams
{
  std::vector<RimShape> shapes;  // the rims looked for, e.g. those of a catalogue's bowls
  double xMin = -0.5;
  double xMax = 0.5;
  double yMin = -0.5;
  double yMax = 0.5;
  double stepM = 0.004;         // first pass; good places are then refined to 1 mm
  double radiusSlackM = 0.003;  // the radius may differ from the shape's by this much
  double minGradient = 6.0;
  double minEdgeFraction = 0.6;
  double hiddenCost = 0.3;
  double maxHidden = 0.5;
  double weakestPartBonus = 0.2;
  double contrastBonus = 0.2;
  double contrastFull = 20.0;
  int points = 48;  // on the circle
  int maxMatchesPerShape = 8;
};

// Looks for rims the way findBoxesByEdges looks for boxes: each shape's circle is drawn at every
// place and compared with the image's edges. Matches of one shape are at least a diameter apart;
// matches of different shapes may share a place (a smaller circle also fits inside a larger
// bowl), and it is for the caller to choose among shapes. Best first.
std::vector<RimEdgeMatch> findRimsByEdges(
  const cv::Mat & bgr, const PinholeCamera & camera, double tableZ, const cv::Mat & hidden,
  const RimEdgeSearchParams & params);

// Of several objects known to be of one shape: the shape whose best `count` matches score
// highest in sum, taking the largest shape among those within `tie` of the best (a smaller
// circle fits inside a larger bowl about as well). Returns those matches; empty when no shape
// has that many.
std::vector<RimEdgeMatch> sameShapeRims(
  const std::vector<RimEdgeMatch> & matches, const std::vector<RimShape> & shapes, int count,
  double tie = 0.15);

}  // namespace cl_rgb
