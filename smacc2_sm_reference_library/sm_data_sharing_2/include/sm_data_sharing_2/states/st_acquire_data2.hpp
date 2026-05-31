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
#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_store_data2.hpp"

namespace sm_data_sharing_2
{
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// Second substate of SsMission. CbStoreData2 writes targetPosition into the superstate.
// The initialPosition written in StAcquireData1 is still present — SsMission was never
// destroyed during the StAcquireData1 -> StAcquireData2 transition.
struct StAcquireData2 : smacc2::SmaccState<StAcquireData2, SsMission>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StProcessData, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StProcessData, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrData, cl_data::CbStoreData2>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    auto & ss = this->context<SsMission>();
    RCLCPP_INFO(
      getLogger(),
      "[StAcquireData2] Entered. SsMission initial position still set: %s "
      "(auto-advance in 5s or press N)",
      ss.initialPosition.has_value() ? "yes" : "no");
  }
};

}  // namespace sm_data_sharing_2
