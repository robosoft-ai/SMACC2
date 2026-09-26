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

#pragma once

#include <cl_px4_mr/client_behaviors/cb_hold_position.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: hover at the far end of the route before turning back (no yaw scan:
// sweeping the safety cone across the cave walls would trip the guard)
struct StHoldAtTurnaround : smacc2::SmaccState<StHoldAtTurnaround, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbHoldPosition, OrPx4>, StTurnAround, SUCCESS>,
    Transition<EvCbFailure<CbHoldPosition, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbHoldPosition>(kTurnaroundHoldS);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StHoldAtTurnaround: holding %.0f s at the far end", kTurnaroundHoldS);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
