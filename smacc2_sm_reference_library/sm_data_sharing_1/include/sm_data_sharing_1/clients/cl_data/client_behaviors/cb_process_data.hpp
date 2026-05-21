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

#include <cmath>
#include <smacc2/smacc.hpp>
#include "../components/cp_mission_data.hpp"

namespace sm_data_sharing_1
{
namespace cl_data
{

// Reads both positions stored in earlier states and computes path info.
// This demonstrates that CpMissionData persists across state transitions —
// data written in StAcquireData1 and StAcquireData2 is still available here.
class CbProcessData : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    CpMissionData * missionData;
    this->requiresComponent(missionData);

    if (!missionData->initialPosition.has_value() || !missionData->targetPosition.has_value())
    {
      RCLCPP_ERROR(getLogger(), "[CbProcessData] Missing position data in component.");
      return;
    }

    const auto & from = missionData->initialPosition.value();
    const auto & to = missionData->targetPosition.value();
    const double dist = std::hypot(to.x - from.x, to.y - from.y);

    RCLCPP_INFO(
      getLogger(),
      "[CbProcessData] Generating path: (%.1f, %.1f) -> (%.1f, %.1f), distance: %.2f m",
      from.x, from.y, to.x, to.y, dist);

    // Reset for next cycle
    missionData->initialPosition.reset();
    missionData->targetPosition.reset();
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_1
