// Copyright 2025 Robosoft Inc.
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

#include <smacc2/smacc.hpp>
#include "../components/cp_mission_data.hpp"

namespace sm_data_sharing_1
{
namespace cl_data
{

// Simulates acquiring a target AGV position and storing it in the shared component.
// Note: CpMissionData lives on ClData (OrData), but requiresComponent() searches
// all clients across all orthogonals — so this same call works from any orthogonal.
class CbStoreData2 : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    CpMissionData * missionData;
    this->requiresComponent(missionData);

    missionData->targetPosition = Position2D{8.0, 5.0};

    RCLCPP_INFO(
      getLogger(), "[CbStoreData2] Stored target position: (%.1f, %.1f)",
      missionData->targetPosition->x, missionData->targetPosition->y);
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_1
