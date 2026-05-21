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
#include "sm_data_sharing_2/superstates/ss_mission.hpp"

namespace sm_data_sharing_2
{
namespace cl_data
{

// Reads both positions from SsMission and computes path info.
// Both values were written in earlier states — they persist here because
// SsMission (and its member data) is not destroyed between substates.
// See CbStoreData1 for the full explanation of the getParentState() pattern.
class CbProcessData : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    auto * ss = dynamic_cast<SsMission *>(this->getCurrentState()->getParentState());

    if (!ss)
    {
      RCLCPP_ERROR(getLogger(), "[CbProcessData] Could not reach SsMission via getParentState().");
      return;
    }

    if (!ss->initialPosition.has_value() || !ss->targetPosition.has_value())
    {
      RCLCPP_ERROR(getLogger(), "[CbProcessData] Missing position data in SsMission.");
      return;
    }

    const auto & from = ss->initialPosition.value();
    const auto & to = ss->targetPosition.value();
    const double dist = std::hypot(to.x - from.x, to.y - from.y);

    RCLCPP_INFO(
      getLogger(),
      "[CbProcessData] Generating path: (%.1f, %.1f) -> (%.1f, %.1f), distance: %.2f m",
      from.x, from.y, to.x, to.y, dist);

    // Reset for next cycle. When SsMission re-enters from outside, this
    // happens automatically — member variables are re-initialized on construction.
    ss->initialPosition.reset();
    ss->targetPosition.reset();
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_2
