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
#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_store_data1.hpp"

namespace sm_data_sharing_2
{
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// Initial substate of SsMission. CbStoreData1 writes initialPosition into the superstate.
// In a state class, the superstate is reachable with this->context<SsMission>() —
// the same SsMission instance that CbStoreData1 accesses via getParentState().
struct StAcquireData1 : smacc2::SmaccState<StAcquireData1, SsMission>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StAcquireData2, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAcquireData2, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrData, cl_data::CbStoreData1>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    // context<SsMission>() is the state-class way to access the same data
    // that CbStoreData1 reaches via getParentState().
    auto & ss = this->context<SsMission>();
    RCLCPP_INFO(
      getLogger(),
      "[StAcquireData1] Entered. SsMission initial position set: %s "
      "(auto-advance in 5s or press N)",
      ss.initialPosition.has_value() ? "yes" : "no");
  }
};

}  // namespace sm_data_sharing_2
