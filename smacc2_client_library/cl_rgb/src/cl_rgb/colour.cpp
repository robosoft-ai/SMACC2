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

#include <cl_rgb/colour.hpp>

#include <cmath>
#include <limits>

namespace cl_rgb
{

const std::vector<NamedColour> & basicColours()
{
  static const std::vector<NamedColour> colours = {
    {"red", {53.2, 80.1, 67.2}},     {"orange", {74.9, 23.9, 78.9}},
    {"yellow", {97.1, -21.6, 94.5}}, {"green", {87.7, -86.2, 83.2}},
    {"cyan", {91.1, -48.1, -14.1}},  {"blue", {32.3, 79.2, -107.9}},
    {"purple", {29.8, 58.9, -36.5}}, {"magenta", {60.3, 98.3, -60.8}},
    {"pink", {83.6, 24.1, 3.3}},     {"brown", {37.5, 26.4, 40.9}},
  };
  return colours;
}

std::string colourName(const cv::Vec3d & lab)
{
  const double chroma = std::hypot(lab[1], lab[2]);
  if (chroma < 12.0)
  {
    return lab[0] < 25.0 ? "black" : (lab[0] > 75.0 ? "white" : "gray");
  }
  std::string best;
  double bestCost = std::numeric_limits<double>::max();
  for (const auto & colour : basicColours())
  {
    const double cost = colourCost(lab, colour.name);
    if (cost < bestCost)
    {
      bestCost = cost;
      best = colour.name;
    }
  }
  return best;
}

double colourCost(const cv::Vec3d & lab, const std::string & name)
{
  for (const auto & colour : basicColours())
  {
    if (colour.name != name)
    {
      continue;
    }
    const double chroma = std::hypot(lab[1], lab[2]);
    double difference =
      std::abs(std::atan2(lab[2], lab[1]) - std::atan2(colour.lab[2], colour.lab[1]));
    if (difference > CV_PI)
    {
      difference = 2.0 * CV_PI - difference;
    }
    // Hue decides; lightness and chroma separate the words that share a hue (orange and
    // brown, red and pink, blue and purple).
    return difference * 60.0 + 0.5 * std::abs(lab[0] - colour.lab[0]) +
           0.25 * std::abs(chroma - std::hypot(colour.lab[1], colour.lab[2]));
  }
  return std::numeric_limits<double>::infinity();
}

namespace
{
void assign(
  const std::vector<std::vector<double>> & cost, size_t name, double sum,
  std::vector<int> & current, std::vector<bool> & used, double & bestSum, std::vector<int> & best)
{
  if (sum >= bestSum)
  {
    return;
  }
  if (name == cost.size())
  {
    bestSum = sum;
    best = current;
    return;
  }
  for (size_t i = 0; i < used.size(); ++i)
  {
    if (!used[i])
    {
      used[i] = true;
      current[name] = static_cast<int>(i);
      assign(cost, name + 1, sum + cost[name][i], current, used, bestSum, best);
      used[i] = false;
    }
  }
}
}  // namespace

std::vector<int> matchColours(
  const std::vector<cv::Vec3d> & seen, const std::vector<std::string> & names)
{
  if (seen.size() < names.size() || names.empty())
  {
    return {};
  }
  std::vector<std::vector<double>> cost(names.size(), std::vector<double>(seen.size(), 0.0));
  for (size_t n = 0; n < names.size(); ++n)
  {
    for (size_t i = 0; i < seen.size(); ++i)
    {
      cost[n][i] = colourCost(seen[i], names[n]);
    }
  }
  std::vector<int> current(names.size(), -1);
  std::vector<int> best;
  std::vector<bool> used(seen.size(), false);
  double bestSum = std::numeric_limits<double>::max();
  assign(cost, 0, 0.0, current, used, bestSum, best);
  return best;
}

}  // namespace cl_rgb
