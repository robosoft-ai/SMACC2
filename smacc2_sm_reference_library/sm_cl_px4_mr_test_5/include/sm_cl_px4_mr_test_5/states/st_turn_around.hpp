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

#include <cl_px4_mr/client_behaviors/cb_yaw_rotate.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: yaw in place at the far end so the retrace starts facing home. A pure
// rotation: no translation, so the vehicle cannot wander into the passage
// walls while its nose (and the lidar cone) sweeps round.
struct StTurnAround : smacc2::SmaccState<StTurnAround, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbYawRotate, OrPx4>, StFlyInbound, SUCCESS>,
    Transition<EvCbFailure<CbYawRotate, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbYawRotate>(kTurnaroundYawRad, true);
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbYawRotate>()->setTimeout(kTurnTimeout);
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StTurnAround: yawing %.0f deg in place", kTurnaroundYawRad * 180.0 / M_PI);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
