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

#include <cl_px4_mr/components/cp_kml_mission_loader.hpp>

#include <tinyxml2.h>

#include <fstream>
#include <sstream>

namespace cl_px4_mr
{

namespace
{

// "gx:LineString" -> "LineString"
std::string localName(const char * qualified)
{
  if (qualified == nullptr)
  {
    return "";
  }
  std::string name(qualified);
  const auto colon = name.rfind(':');
  return colon == std::string::npos ? name : name.substr(colon + 1);
}

const tinyxml2::XMLElement * firstChildByLocalName(
  const tinyxml2::XMLElement * parent, const std::string & wanted)
{
  for (const tinyxml2::XMLElement * child = parent->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    if (localName(child->Name()) == wanted)
    {
      return child;
    }
  }
  return nullptr;
}

// depth-first search for the first <LineString> with a non-empty <coordinates>
const tinyxml2::XMLElement * findFirstLineStringCoordinates(const tinyxml2::XMLElement * element)
{
  if (element == nullptr)
  {
    return nullptr;
  }

  if (localName(element->Name()) == "LineString")
  {
    const tinyxml2::XMLElement * coords = firstChildByLocalName(element, "coordinates");
    if (coords != nullptr && coords->GetText() != nullptr)
    {
      return coords;
    }
  }

  for (const tinyxml2::XMLElement * child = element->FirstChildElement(); child != nullptr;
       child = child->NextSiblingElement())
  {
    const tinyxml2::XMLElement * found = findFirstLineStringCoordinates(child);
    if (found != nullptr)
    {
      return found;
    }
  }
  return nullptr;
}

// KML coordinate tuples are "lon,lat[,alt]" separated by whitespace
bool parseCoordinates(const std::string & text, std::vector<GeoPoint> & out, std::string & error)
{
  std::istringstream tokens(text);
  std::string token;
  while (tokens >> token)
  {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream fieldStream(token);
    while (std::getline(fieldStream, field, ','))
    {
      fields.push_back(field);
    }

    if (fields.size() < 2)
    {
      error = "coordinate tuple '" + token + "' has fewer than 2 fields";
      return false;
    }

    try
    {
      GeoPoint p;
      p.lon = std::stod(fields[0]);
      p.lat = std::stod(fields[1]);
      p.alt = fields.size() >= 3 && !fields[2].empty() ? std::stod(fields[2]) : 0.0;
      out.push_back(p);
    }
    catch (const std::exception & e)
    {
      error = "coordinate tuple '" + token + "' is not numeric: " + e.what();
      return false;
    }
  }

  if (out.empty())
  {
    error = "LineString has no coordinate tuples";
    return false;
  }
  return true;
}

std::vector<GeoPoint> parseDocument(tinyxml2::XMLDocument & doc, std::string & error)
{
  std::vector<GeoPoint> points;
  const tinyxml2::XMLElement * coords = findFirstLineStringCoordinates(doc.RootElement());
  if (coords == nullptr)
  {
    error = "no LineString/coordinates element found";
    return points;
  }

  if (!parseCoordinates(coords->GetText(), points, error))
  {
    points.clear();
  }
  return points;
}

}  // namespace

CpKmlMissionLoader::CpKmlMissionLoader() {}

CpKmlMissionLoader::~CpKmlMissionLoader() {}

void CpKmlMissionLoader::onInitialize()
{
  RCLCPP_INFO(getLogger(), "CpKmlMissionLoader: ready (first LineString backbone only)");
}

std::vector<GeoPoint> CpKmlMissionLoader::parseKmlString(const std::string & xml, std::string & error)
{
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS)
  {
    error = std::string("XML parse error: ") + (doc.ErrorStr() ? doc.ErrorStr() : "unknown");
    return {};
  }
  return parseDocument(doc, error);
}

KmlLoadResult CpKmlMissionLoader::loadFile(const std::string & absolutePath)
{
  KmlLoadResult result;

  tinyxml2::XMLDocument doc;
  const tinyxml2::XMLError status = doc.LoadFile(absolutePath.c_str());
  if (status != tinyxml2::XML_SUCCESS)
  {
    result.error = std::string("cannot load '") + absolutePath + "': " +
                   (doc.ErrorStr() ? doc.ErrorStr() : "unknown error");
    RCLCPP_WARN(getLogger(), "CpKmlMissionLoader: %s", result.error.c_str());
    return result;
  }

  std::string error;
  std::vector<GeoPoint> points = parseDocument(doc, error);
  if (points.empty())
  {
    result.error = "'" + absolutePath + "': " + error;
    RCLCPP_WARN(getLogger(), "CpKmlMissionLoader: %s", result.error.c_str());
    return result;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    mission_ = points;
    source_ = absolutePath;
  }

  result.ok = true;
  result.pointCount = points.size();
  RCLCPP_INFO(
    getLogger(), "CpKmlMissionLoader: loaded %zu backbone points from '%s' (first %.6f,%.6f last %.6f,%.6f)",
    points.size(), absolutePath.c_str(), points.front().lat, points.front().lon,
    points.back().lat, points.back().lon);
  return result;
}

void CpKmlMissionLoader::setMission(std::vector<GeoPoint> points, const std::string & source)
{
  std::lock_guard<std::mutex> lock(mutex_);
  mission_ = std::move(points);
  source_ = source;
}

bool CpKmlMissionLoader::hasMission() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !mission_.empty();
}

std::vector<GeoPoint> CpKmlMissionLoader::getMission() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mission_;
}

std::string CpKmlMissionLoader::getSource() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return source_;
}

void CpKmlMissionLoader::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  mission_.clear();
  source_.clear();
}

}  // namespace cl_px4_mr
