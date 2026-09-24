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

#include <cl_px4_mr/client_behaviors/cb_loiter.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/states/st_figure_eight_1.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// TAIL PATTERN STATE: loiter circles about the figure-eight centroid, then
// head home to land
struct StLoiterCentroid : smacc2::SmaccState<StLoiterCentroid, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLoiter, OrPx4>, StGoToLandingZone, SUCCESS>,
    Transition<EvCbFailure<CbLoiter, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLoiter>(kCentroidLoiterCount, kLoiterRadiusM);
  }

  void runtimeConfigure()
  {
    const NedXY c = StFigureEight1::centre();
    auto * cb = this->getClientBehavior<OrPx4, CbLoiter>();

    FlightPatternLoiterParams p = cb->params();
    p.centerX = c.x;
    p.centerY = c.y;
    p.altitudeAgl = kMissionAltitudeM;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = kPatternSpeedMps;
    f.leash = kPatternLeashM;
    cb->setFollowerParams(f);

    const float circumference =
      2.0f * kPi * kLoiterRadiusM * kCentroidLoiterCount;
    cb->setTimeout(patternTimeout(circumference, kPatternSpeedMps));

    RCLCPP_INFO(
      getLogger(), "StLoiterCentroid: %d x r=%.0f m about NED (%.1f, %.1f)",
      kCentroidLoiterCount, kLoiterRadiusM, static_cast<double>(c.x),
      static_cast<double>(c.y));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
