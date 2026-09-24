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

#include <cl_px4_mr/client_behaviors/cb_sine_wave_horizontal.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_grid_pattern_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_search_1.hpp>


namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// TRANSIT STATE: horizontal sine-wave leg from the grid to the sector-search datum (P6); the rotated triple
// runs there back to back with no transit between
struct StTransitToVSSearch : smacc2::SmaccState<StTransitToVSSearch, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSineWaveHorizontal, OrPx4>, SsVSSearch1, SUCCESS>,
    Transition<EvCbFailure<CbSineWaveHorizontal, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSineWaveHorizontal>();
  }

  void runtimeConfigure()
  {
    const NedXY from = SsGridPattern1::exit();
    const NedXY to = SsVSSearch1::entry();
    const float lengthM = distanceM(from, to);

    auto * cb = this->getClientBehavior<OrPx4, CbSineWaveHorizontal>();
    FlightPatternSineWaveHorizontalParams p = cb->params();
    p.endX = to.x;
    p.endY = to.y;
    p.altitudeAgl = kMissionAltitudeM;
    p.amplitude = kSineAmplitudeM;
    p.wavelength = kSineWavelengthM;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = kCruiseSpeedMps;
    f.leash = kCruiseLeashM;
    cb->setFollowerParams(f);

    cb->setTimeout(transitTimeout(lengthM));

    RCLCPP_INFO(
      getLogger(), "StTransitToVSSearch: -> P6 entry NED (%.1f, %.1f), %.0f m", static_cast<double>(to.x),
      static_cast<double>(to.y), static_cast<double>(lengthM));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
