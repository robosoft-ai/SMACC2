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

#include <smacc2/smacc.hpp>

#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <config/mission_constants.hpp>

#include <cmath>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: mission abort leg - return to the takeoff point at mission altitude,
// then land whatever happens
struct StReturnHome : smacc2::SmaccState<StReturnHome, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, MsLanding, SUCCESS>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, MsLanding, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedXY home = island();
    configure_orthogonal<OrPx4, CbGoToLocation>(home.x, home.y, -kMissionAltitudeM);
  }

  void runtimeConfigure()
  {
    // bound the return by the farthest place in the mission (a chain pearl,
    // one spacing along the tangent at the ring radius) plus a pattern's extent
    const float farthestM =
      std::hypot(kDemoRingRadiusM, kDemoChainSpacingM) + kPatternSquareSideM;
    this->getClientBehavior<OrPx4, CbGoToLocation>()->setTimeout(transitTimeout(farthestM));
  }

  void onEntry()
  {
    RCLCPP_WARN(getLogger(), "StReturnHome: MISSION ABORTED - returning to home");
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
