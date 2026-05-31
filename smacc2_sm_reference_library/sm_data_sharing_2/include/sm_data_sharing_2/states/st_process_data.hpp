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
#include "sm_data_sharing_2/clients/cl_data/client_behaviors/cb_process_data.hpp"

namespace sm_data_sharing_2
{
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace smacc2::default_transition_tags;

// Final substate of SsMission. CbProcessData reads both positions and computes
// path info, then clears the data. The loop transition back to StAcquireData1
// stays inside SsMission — the superstate is never re-entered, so member variables
// are NOT reset by the framework. CbProcessData handles the manual reset.
struct StProcessData : smacc2::SmaccState<StProcessData, SsMission>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE - loops back within SsMission
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StAcquireData1, SUCCESS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StAcquireData1, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrData, cl_data::CbProcessData>();
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void onEntry()
  {
    auto & ss = this->context<SsMission>();
    RCLCPP_INFO(
      getLogger(),
      "[StProcessData] Entered. Both positions set: initial=%s target=%s "
      "(auto-advance in 5s or press N)",
      ss.initialPosition.has_value() ? "yes" : "no",
      ss.targetPosition.has_value() ? "yes" : "no");
  }
};

}  // namespace sm_data_sharing_2
