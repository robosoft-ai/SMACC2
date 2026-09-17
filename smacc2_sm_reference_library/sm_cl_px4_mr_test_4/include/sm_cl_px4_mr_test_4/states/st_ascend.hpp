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
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/railway/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/railway/railway_events.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: climb from the takeoff altitude to the mission altitude
struct StAscend : smacc2::SmaccState<StAscend, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbAscendToAltitude, OrPx4>, StSpiralOffIsland, SUCCESS>,
    Transition<EvCbFailure<CbAscendToAltitude, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbAscendToAltitude>(
      railway::kMissionAltitudeM, railway::kAscendClimbRateMps);
  }

  void runtimeConfigure()
  {
    const auto & plan = this->context<MsInFlight>().plan;
    auto * cb = this->getClientBehavior<OrPx4, CbAscendToAltitude>();
    const float climbSeconds =
      (railway::kMissionAltitudeM - railway::kTakeoffAltitudeM) / railway::kAscendClimbRateMps;
    cb->setTimeout(std::chrono::seconds(
      static_cast<long>(climbSeconds * railway::kTimeoutMarginFactor + railway::kTimeoutBaseS)));
    if (!plan.valid)
    {
      RCLCPP_ERROR(getLogger(), "StAscend: mission plan invalid - the railway will land");
    }
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StAscend: climbing to %.1f m", railway::kMissionAltitudeM);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
