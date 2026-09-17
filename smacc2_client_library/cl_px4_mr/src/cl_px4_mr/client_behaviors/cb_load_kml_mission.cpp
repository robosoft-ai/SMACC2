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

#include <cl_px4_mr/client_behaviors/cb_load_kml_mission.hpp>
#include <cl_px4_mr/components/cp_kml_mission_loader.hpp>

namespace cl_px4_mr
{

CbLoadKmlMission::CbLoadKmlMission(
  std::string absoluteKmlPath, std::vector<GeoPoint> fallbackBackbone)
: path_(std::move(absoluteKmlPath)), fallback_(std::move(fallbackBackbone))
{
}

void CbLoadKmlMission::onEntry()
{
  if (kmlLoader_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "CbLoadKmlMission: CpKmlMissionLoader component missing");
    this->postPx4Failure();
    return;
  }

  RCLCPP_INFO(getLogger(), "CbLoadKmlMission: loading backbone from '%s'", path_.c_str());
  const KmlLoadResult result = kmlLoader_->loadFile(path_);

  if (result.ok)
  {
    RCLCPP_INFO(
      getLogger(), "CbLoadKmlMission: %zu backbone points loaded from file", result.pointCount);
    this->postPx4Success();
    return;
  }

  if (!fallback_.empty())
  {
    kmlLoader_->setMission(fallback_, "fallback:compiled-in");
    RCLCPP_WARN(
      getLogger(),
      "CbLoadKmlMission: KML unavailable (%s) - using compiled-in fallback backbone (%zu points)",
      result.error.c_str(), fallback_.size());
    this->postPx4Success();
    return;
  }

  RCLCPP_ERROR(
    getLogger(), "CbLoadKmlMission: KML unavailable (%s) and no fallback backbone supplied",
    result.error.c_str());
  this->postPx4Failure();
}

void CbLoadKmlMission::onExit() {}

}  // namespace cl_px4_mr
