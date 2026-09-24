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
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_square_spiral_2.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_lawnmower_1.hpp>


namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// TRANSIT STATE: vertical sine-wave leg from square spiral 2 to the entry of lawnmower 1 (P3)
struct StTransitToLawnmower1 : smacc2::SmaccState<StTransitToLawnmower1, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSineWaveVertical, OrPx4>, SsLawnmower1, SUCCESS>,
    Transition<EvCbFailure<CbSineWaveVertical, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSineWaveVertical>();
  }

  void runtimeConfigure()
  {
    const NedXY from = SsSquareSpiral2::exit();
    const NedXY to = SsLawnmower1::entry();
    const float lengthM = distanceM(from, to);

    auto * cb = this->getClientBehavior<OrPx4, CbSineWaveVertical>();
    FlightPatternSineWaveVerticalParams p = cb->params();
    p.endX = to.x;
    p.endY = to.y;
    p.baseAltitudeAgl = kMissionAltitudeM;
    p.amplitude = kSineAmplitudeM;
    p.wavelength = kSineWavelengthM;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = kCruiseSpeedMps;
    f.leash = kCruiseLeashM;
    cb->setFollowerParams(f);

    cb->setTimeout(transitTimeout(lengthM));

    RCLCPP_INFO(
      getLogger(), "StTransitToLawnmower1: -> P3 entry NED (%.1f, %.1f), %.0f m", static_cast<double>(to.x),
      static_cast<double>(to.y), static_cast<double>(lengthM));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
