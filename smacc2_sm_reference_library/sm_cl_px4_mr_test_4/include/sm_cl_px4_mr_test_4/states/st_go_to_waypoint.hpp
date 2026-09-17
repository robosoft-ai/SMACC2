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

#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// NAV STATE: straight go-to leg to the plan's current target node (no sine
// wave); returns to StRailway
struct StGoToWaypoint : smacc2::SmaccState<StGoToWaypoint, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StRailway, NEXT>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbGoToLocation>(0.0f, 0.0f, -railway::kMissionAltitudeM);
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    const auto & leg = plan.current();
    const auto & to = plan.currentTargetNode();
    auto * cb = this->getClientBehavior<OrPx4, CbGoToLocation>();
    cb->setTarget(to.entryX, to.entryY, plan.altitudeNedZ);
    cb->setTimeout(railway::transitTimeout(std::max(leg.lengthM, 50.0f)));
    RCLCPP_INFO(
      getLogger(), "StGoToWaypoint: leg L%d -> '%s' NED (%.1f, %.1f), %.0f m", leg.index,
      to.name.c_str(), static_cast<double>(to.entryX), static_cast<double>(to.entryY),
      static_cast<double>(leg.lengthM));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
