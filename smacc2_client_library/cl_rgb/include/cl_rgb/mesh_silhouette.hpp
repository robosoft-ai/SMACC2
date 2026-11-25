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

#include <optional>
#include <string>
#include <vector>

namespace cl_rgb
{

// A triangle mesh: three vertices per triangle, in the mesh's own frame, in metres.
struct Mesh
{
  std::vector<cv::Vec3d> vertices;
};

// Reads a binary STL file; nothing when it cannot be read.
std::optional<Mesh> loadStl(const std::string & path);

// The file of a link's first mesh in the text of a URDF, without a leading "file://"; empty
// when the link or a mesh is not found.
std::string urdfMeshPath(const std::string & urdf, const std::string & link);

// Paints into the mask (8 bit, one channel) the pixels the mesh covers as the camera sees it.
// rotation and translation take the mesh's frame to the world frame. This is how a robot's own
// parts are left out of a picture: from where they are known to be, not from how they look.
void drawMesh(
  cv::Mat & mask, const Mesh & mesh, const cv::Matx33d & rotation, const cv::Vec3d & translation,
  const PinholeCamera & camera);

}  // namespace cl_rgb
