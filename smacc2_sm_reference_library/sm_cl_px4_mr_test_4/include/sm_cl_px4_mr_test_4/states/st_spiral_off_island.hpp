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

#include <cl_px4_mr/client_behaviors/cb_spiral.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: outward spiral off the island after reaching mission altitude, then
// hand over to the railway
struct StSpiralOffIsland : smacc2::SmaccState<StSpiralOffIsland, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSpiral, OrPx4>, StRailway, NEXT>,
    Transition<EvCbFailure<CbSpiral, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSpiral>();
  }

  void runtimeConfigure()
  {
    auto * cb = this->getClientBehavior<OrPx4, CbSpiral>();
    FlightPatternSpiralParams p = cb->params();  // centre = current position
    p.altitudeAgl = railway::kMissionAltitudeM;
    p.startRadius = 0.0f;
    p.spacing = railway::kTrackSpacingM;
    p.endRadius = railway::kSpiralOffIslandTurns * railway::kTrackSpacingM;
    p.direction = railway::kSpiralOffIslandDirection;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = railway::kCruiseSpeedMps;
    f.leash = railway::kCruiseLeashM;
    cb->setFollowerParams(f);

    cb->setTimeout(railway::patternTimeout(flightPatternSpiralLength(p), railway::kCruiseSpeedMps));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StSpiralOffIsland: spiralling out %.1f turns to %.0f m", railway::kSpiralOffIslandTurns,
      railway::kSpiralOffIslandTurns * railway::kTrackSpacingM);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
