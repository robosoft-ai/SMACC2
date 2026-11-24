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

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace cl_rgb
{

// A colour word and where it lies in Lab (L 0 to 100, a and b around 0).
struct NamedColour
{
  std::string name;
  cv::Vec3d lab;
};

// The everyday colour words, each at the Lab value of its pure sRGB colour.
const std::vector<NamedColour> & basicColours();

// The word whose colour is nearest to the given one. Colours with little chroma are told
// apart by lightness (black, gray, white); the others by hue, so that shading does not turn
// one colour into another.
std::string colourName(const cv::Vec3d & lab);

// How far a colour is from a colour word: 0 for the word's own colour, larger the further.
// Infinite for a word that is not among basicColours().
double colourCost(const cv::Vec3d & lab, const std::string & name);

// Gives each of the named colours one of the seen colours, no seen colour twice, so that the
// costs add up to the least. For a scene whose colours are known by name beforehand: a dark
// blue is then "blue" because nothing else there is, though alone it would be called purple.
// Returns the index into `seen` for each name, or nothing when there are fewer seen than named.
std::vector<int> matchColours(
  const std::vector<cv::Vec3d> & seen, const std::vector<std::string> & names);

}  // namespace cl_rgb
