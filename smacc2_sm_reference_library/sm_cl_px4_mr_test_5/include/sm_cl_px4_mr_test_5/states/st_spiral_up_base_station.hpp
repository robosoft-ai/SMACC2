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

#include <cl_px4_mr/client_behaviors/cb_spiral_up.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: finale, part 3 - climb around the base station on a helix
struct StSpiralUpBaseStation : smacc2::SmaccState<StSpiralUpBaseStation, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSpiralUp, OrPx4>, StOrbitAtTop, SUCCESS>,
    Transition<EvCbFailure<CbSpiralUp, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSpiralUp>(finaleSpiralParams());
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrPx4, CbSpiralUp>()->setTimeout(
      legTimeout(finaleSpiralLengthM(), kFinaleAngularRateRadS * kFinaleRadiusM));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StSpiralUpBaseStation: helix %.0f -> %.0f m, %.0f m per orbit, radius %.0f m",
      static_cast<double>(kFinaleAltitudeM), static_cast<double>(kFinaleTopAltitudeM),
      static_cast<double>(kFinaleClimbPerOrbitM), static_cast<double>(kFinaleRadiusM));
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
