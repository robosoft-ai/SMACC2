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
#include "sm_data_sharing_2/superstates/ss_mission.hpp"

namespace sm_data_sharing_2
{
namespace cl_data
{

// Stores the target position into SsMission's data fields.
// See CbStoreData1 for the full explanation of the getParentState() pattern.
class CbStoreData2 : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    auto * ss = dynamic_cast<SsMission *>(this->getCurrentState()->getParentState());

    if (!ss)
    {
      RCLCPP_ERROR(getLogger(), "[CbStoreData2] Could not reach SsMission via getParentState().");
      return;
    }

    ss->targetPosition = Position2D{8.0, 5.0};

    RCLCPP_INFO(
      getLogger(), "[CbStoreData2] Stored target position in SsMission: (%.1f, %.1f)",
      ss->targetPosition->x, ss->targetPosition->y);
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_2
