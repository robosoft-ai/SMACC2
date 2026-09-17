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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>
#include <cl_px4_mr/utils/geo_utils.hpp>
#include <smacc2/smacc.hpp>

#include <string>
#include <vector>

namespace cl_px4_mr
{

class CpKmlMissionLoader;

// Loads the mission backbone (first LineString) from an ABSOLUTE KML path into
// CpKmlMissionLoader. The config file is a convenience: if it is absent or
// unparseable and a fallback backbone was supplied, the fallback is stored
// instead and the behavior still succeeds (with a WARN). Failure is posted
// only when nothing usable is available.
class CbLoadKmlMission : public CbPx4ClientBehaviorBase
{
public:
  explicit CbLoadKmlMission(
    std::string absoluteKmlPath, std::vector<GeoPoint> fallbackBackbone = {});

  void setFilePath(const std::string & absoluteKmlPath) { path_ = absoluteKmlPath; }
  void setFallback(std::vector<GeoPoint> fallbackBackbone)
  {
    fallback_ = std::move(fallbackBackbone);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(kmlLoader_, smacc2::ComponentRequirement::SOFT);
    CbPx4ClientBehaviorBase::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  void onEntry() override;
  void onExit() override;

private:
  CpKmlMissionLoader * kmlLoader_ = nullptr;
  std::string path_;
  std::vector<GeoPoint> fallback_;
};

}  // namespace cl_px4_mr
