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
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: loiter over the landing zone before landing
struct StLoiterHotel : smacc2::SmaccState<StLoiterHotel, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLoiter, OrPx4>, StPreLandDescent, SUCCESS>,
    Transition<EvCbFailure<CbLoiter, OrPx4>, StPreLandDescent, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLoiter>(railway::kLandingLoiterCount, railway::kLoiterRadiusM);
  }

  void runtimeConfigure()
  {
    auto * cb = this->getClientBehavior<OrPx4, CbLoiter>();
    FlightPatternLoiterParams p = cb->params();
    p.altitudeAgl = railway::kMissionAltitudeM;
    cb->setParams(p);

    PathFollowerParams f = cb->followerParams();
    f.groundSpeed = railway::kPatternSpeedMps;
    f.leash = railway::kPatternLeashM;
    cb->setFollowerParams(f);

    const float circumference =
      2.0f * railway::kPi * railway::kLoiterRadiusM * std::max(railway::kLandingLoiterCount, 1);
    cb->setTimeout(railway::patternTimeout(circumference, railway::kPatternSpeedMps));
  }

  void onEntry()
  {
    RCLCPP_INFO(
      getLogger(), "StLoiterHotel: loitering %d x r=%.0f m", railway::kLandingLoiterCount,
      railway::kLoiterRadiusM);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
