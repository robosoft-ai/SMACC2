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

#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <config/mission_constants.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: precise pre-landing descent. PX4's AUTO_LAND holds the horizontal
// position only loosely over a long descent (3 m of drift seen from 30 m), so
// the state machine descends in offboard to just above the pad while holding
// the exact landing point, and hands over to AUTO_LAND for the last metre.
struct StPreLandDescent : smacc2::SmaccState<StPreLandDescent, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbAscendToAltitude, OrPx4>, MsLanding, SUCCESS>,
    Transition<EvCbFailure<CbAscendToAltitude, OrPx4>, MsLanding, ABORT>  // land anyway
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbAscendToAltitude>(
      kPreLandAltitudeM, kPreLandDescentRateMps);
  }

  void runtimeConfigure()
  {
    const NedXY lz = island();
    auto * cb = this->getClientBehavior<OrPx4, CbAscendToAltitude>();
    cb->setTargetXy(lz.x, lz.y);

    PathFollowerParams f = cb->followerParams();
    f.arrivalXyTol = kPreLandXyTolM;
    f.arrivalZTol = kPreLandXyTolM;
    cb->setFollowerParams(f);

    const float seconds =
      (kMissionAltitudeM - kPreLandAltitudeM) / kPreLandDescentRateMps;
    cb->setTimeout(std::chrono::seconds(static_cast<long>(
      seconds * kTimeoutMarginFactor + kTimeoutBaseS)));

    RCLCPP_INFO(
      getLogger(), "StPreLandDescent: descending to %.1f m over the island NED (%.1f, %.1f), xy tol %.2f m",
      kPreLandAltitudeM, static_cast<double>(lz.x), static_cast<double>(lz.y),
      kPreLandXyTolM);
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
