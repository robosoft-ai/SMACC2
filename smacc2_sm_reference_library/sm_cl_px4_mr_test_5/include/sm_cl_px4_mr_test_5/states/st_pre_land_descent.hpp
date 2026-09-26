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

#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: precise pre-landing descent over the spawn point in offboard, then
// hand over to AUTO_LAND for the last metre (PX4's land mode drifts
// horizontally over a long descent)
struct StPreLandDescent : smacc2::SmaccState<StPreLandDescent, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbAscendToAltitude, OrPx4>, MsLanding, SUCCESS>,
    Transition<EvCbFailure<CbAscendToAltitude, OrPx4>, MsLanding, ABORT>  // land anyway
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbAscendToAltitude>(kPreLandAltitudeM, kPreLandDescentRateMps);
  }

  void runtimeConfigure()
  {
    auto * cb = this->getClientBehavior<OrPx4, CbAscendToAltitude>();
    cb->setTargetXy(0.0f, 0.0f);

    PathFollowerParams f = cb->followerParams();
    f.arrivalXyTol = kPreLandXyTolM;
    f.arrivalZTol = kPreLandXyTolM;
    cb->setFollowerParams(f);

    cb->setTimeout(legTimeout(kCruiseAltitudeM, kPreLandDescentRateMps));

    RCLCPP_INFO(
      getLogger(), "StPreLandDescent: descending to %.1f m over the spawn point, xy tol %.2f m",
      kPreLandAltitudeM, kPreLandXyTolM);
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
