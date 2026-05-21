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

// Stores the initial position into SsMission's data fields.
//
// Inside a state class, the superstate is accessed via context<SsMission>()
// (inherited from boost::statechart). From a client behavior, context<>() is
// not available, so we navigate the hierarchy with getParentState() instead:
//
//   getCurrentState()           -> StAcquireData1  (ISmaccState*)
//   getCurrentState()
//     ->getParentState()        -> SsMission        (ISmaccState*)
//   dynamic_cast<SsMission*>()  -> SsMission*       (typed pointer)
class CbStoreData1 : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    auto * ss = dynamic_cast<SsMission *>(this->getCurrentState()->getParentState());

    if (!ss)
    {
      RCLCPP_ERROR(getLogger(), "[CbStoreData1] Could not reach SsMission via getParentState().");
      return;
    }

    ss->initialPosition = Position2D{1.0, 2.0};

    RCLCPP_INFO(
      getLogger(), "[CbStoreData1] Stored initial position in SsMission: (%.1f, %.1f)",
      ss->initialPosition->x, ss->initialPosition->y);
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_2
