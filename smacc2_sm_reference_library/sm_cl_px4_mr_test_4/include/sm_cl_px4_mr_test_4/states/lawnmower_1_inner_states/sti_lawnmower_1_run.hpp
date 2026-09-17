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

#include <cl_px4_mr/client_behaviors/cb_lawnmower.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_lawnmower_1.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// INNER STATE: run the pattern at the pin, then return to StRailway
struct StiLawnmower1Run : smacc2::SmaccState<StiLawnmower1Run, SsLawnmower1>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLawnmower, OrPx4>, StRailway, NEXT>,
    Transition<EvCbFailure<CbLawnmower, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLawnmower>();
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    const auto & pin = plan.currentTargetNode();
    const auto & params = this->context<SsLawnmower1>().params;

    auto * cb = this->getClientBehavior<OrPx4, CbLawnmower>();
    FlightPatternLawnmowerParams p = params.pattern;
    p.originX = pin.x;
    p.originY = pin.y;
    cb->setParams(p);
    cb->setFollowerParams(params.follower);
    cb->setTimeout(railway::patternTimeout(flightPatternLawnmowerLength(p), params.follower.groundSpeed));

    RCLCPP_INFO(
      getLogger(), "StiLawnmower1Run: pin '%s' NED (%.1f, %.1f)", pin.name.c_str(),
      static_cast<double>(pin.x), static_cast<double>(pin.y));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
