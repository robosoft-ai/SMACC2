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

#include <cl_px4_mr/client_behaviors/cb_orbit_location.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: finale, part 4 - level laps at the top of the helix, nose toward the tent
struct StOrbitAtTop : smacc2::SmaccState<StOrbitAtTop, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbOrbitLocation, OrPx4>, StReturnToPad, SUCCESS>,
    Transition<EvCbFailure<CbOrbitLocation, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    const NedPoint c = baseStationTentNed();
    configure_orthogonal<OrPx4, CbOrbitLocation>(
      c.x, c.y, kFinaleTopAltitudeM, kFinaleRadiusM, kFinaleAngularRateRadS, kFinaleOrbitsTop);
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbOrbitLocation>()->setTimeout(
      legTimeout(finaleOrbitLengthM(kFinaleOrbitsTop), kFinaleAngularRateRadS * kFinaleRadiusM));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StOrbitAtTop: %d orbits, radius %.0f m, %.0f m up", kFinaleOrbitsTop,
      static_cast<double>(kFinaleRadiusM), static_cast<double>(kFinaleTopAltitudeM));
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
