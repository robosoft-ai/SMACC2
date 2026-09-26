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

#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: finale, part 1 - from the pad (back at cruise height after the retrace)
// climb and fly to the entry point of the orbit circle around the base station
struct StFlyToBaseStation : smacc2::SmaccState<StFlyToBaseStation, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StOrbitBaseStation, SUCCESS>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedPoint p = finaleOrbitEntryNed();
    configure_orthogonal<OrPx4, CbGoToLocation>(p.x, p.y, p.z);
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbGoToLocation>()->setTimeout(
      legTimeout(kFinaleAltitudeM + kFinaleRadiusM + 10.0f, kClimbRateMps));
  }

  void onEntry()
  {
    const NedPoint p = finaleOrbitEntryNed();
    RCLCPP_INFO(
      getLogger(), "StFlyToBaseStation: to the orbit entry NED (%.1f, %.1f, %.1f)",
      static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
