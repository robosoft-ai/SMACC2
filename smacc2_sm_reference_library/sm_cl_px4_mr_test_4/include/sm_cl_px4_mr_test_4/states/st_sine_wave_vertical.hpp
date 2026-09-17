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

#include <cl_px4_mr/client_behaviors/cb_sine_wave_vertical.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_planner.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// NAV STATE: vertical sine-wave transit leg to the plan's current target node; returns to StRailway
struct StSineWaveVertical : smacc2::SmaccState<StSineWaveVertical, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSineWaveVertical, OrPx4>, StRailway, NEXT>,
    Transition<EvCbFailure<CbSineWaveVertical, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSineWaveVertical>();
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    const auto & leg = plan.current();
    const auto & to = plan.currentTargetNode();

    auto * cb = this->getClientBehavior<OrPx4, CbSineWaveVertical>();
    FlightPatternSineWaveVerticalParams p = cb->params();
    p.endX = to.entryX;
    p.endY = to.entryY;
    p.baseAltitudeAgl = railway::kMissionAltitudeM;
    p.amplitude = railway::kSineAmplitudeM;
    p.wavelength = railway::kSineWavelengthM;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = railway::kCruiseSpeedMps;
    f.leash = railway::kCruiseLeashM;
    cb->setFollowerParams(f);

    cb->setTimeout(railway::transitTimeout(leg.lengthM));

    RCLCPP_INFO(
      getLogger(), "StSineWaveVertical: leg L%d %s -> '%s' NED (%.1f, %.1f), %.0f m", leg.index,
      plan.node(leg.fromNode).name.c_str(), to.name.c_str(), static_cast<double>(to.entryX),
      static_cast<double>(to.entryY), static_cast<double>(leg.lengthM));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
