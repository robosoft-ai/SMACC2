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

// STATE: fly to the landing zone (B4 nominal, home in test mode) at mission altitude,
// then loiter (StLoiterHotel) before landing
struct StGoToLandingZone : smacc2::SmaccState<StGoToLandingZone, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StLoiterHotel, SUCCESS>,
    Transition<EvCbFailure<CbGoToLocation, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    // target overridden from the plan in runtimeConfigure
    configure_orthogonal<OrPx4, CbGoToLocation>(0.0f, 0.0f, -railway::kMissionAltitudeM);
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    const auto & lz = plan.landingNode();
    auto * cb = this->getClientBehavior<OrPx4, CbGoToLocation>();
    cb->setTarget(lz.x, lz.y, plan.altitudeNedZ);

    const float lengthM = plan.exhausted() ? 0.0f : plan.current().lengthM;
    cb->setTimeout(railway::transitTimeout(std::max(lengthM, 100.0f)));

    RCLCPP_INFO(
      getLogger(), "StGoToLandingZone: landing zone '%s' at NED (%.1f, %.1f, %.1f)",
      lz.name.c_str(), static_cast<double>(lz.x), static_cast<double>(lz.y),
      static_cast<double>(plan.altitudeNedZ));
  }

  void onEntry() {}

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StGoToLandingZone: leaving");
  }
};

}  // namespace sm_cl_px4_mr_test_4
