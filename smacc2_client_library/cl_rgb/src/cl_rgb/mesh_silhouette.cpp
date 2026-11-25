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

#include <cl_rgb/mesh_silhouette.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>

namespace cl_rgb
{

std::optional<Mesh> loadStl(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  char header[80];
  uint32_t triangles = 0;
  if (
    !file.read(header, sizeof(header)) ||
    !file.read(reinterpret_cast<char *>(&triangles), sizeof(triangles)))
  {
    return std::nullopt;
  }
  Mesh mesh;
  mesh.vertices.reserve(static_cast<size_t>(triangles) * 3);
  for (uint32_t i = 0; i < triangles; ++i)
  {
    // a normal, three vertices, two bytes of attributes
    char record[50];
    if (!file.read(record, sizeof(record)))
    {
      return std::nullopt;
    }
    for (int v = 0; v < 3; ++v)
    {
      float xyz[3];
      std::memcpy(xyz, record + 12 + v * 12, sizeof(xyz));
      mesh.vertices.emplace_back(xyz[0], xyz[1], xyz[2]);
    }
  }
  return mesh;
}

std::string urdfMeshPath(const std::string & urdf, const std::string & link)
{
  const auto start = urdf.find("<link name=\"" + link + "\"");
  if (start == std::string::npos)
  {
    return std::string();
  }
  const auto end = urdf.find("</link>", start);
  const std::string key = "filename=\"";
  const auto name = urdf.find(key, start);
  if (name == std::string::npos || name > end)
  {
    return std::string();
  }
  const auto begin = name + key.size();
  std::string path = urdf.substr(begin, urdf.find('"', begin) - begin);
  const std::string scheme = "file://";
  return path.compare(0, scheme.size(), scheme) == 0 ? path.substr(scheme.size()) : path;
}

void drawMesh(
  cv::Mat & mask, const Mesh & mesh, const cv::Matx33d & rotation, const cv::Vec3d & translation,
  const PinholeCamera & camera)
{
  for (size_t i = 0; i + 2 < mesh.vertices.size(); i += 3)
  {
    cv::Point triangle[3];
    bool seen = true;
    for (int v = 0; v < 3 && seen; ++v)
    {
      const auto pixel = camera.project(rotation * mesh.vertices[i + v] + translation);
      seen = pixel.has_value();
      if (seen)
      {
        // a vertex beside the lens lands far outside the picture: kept within what can be drawn
        triangle[v] = cv::Point(
          static_cast<int>(std::clamp(pixel->x, -8000.0, 8000.0)),
          static_cast<int>(std::clamp(pixel->y, -8000.0, 8000.0)));
      }
    }
    if (seen)
    {
      cv::fillConvexPoly(mask, triangle, 3, 255);
    }
  }
}

}  // namespace cl_rgb
