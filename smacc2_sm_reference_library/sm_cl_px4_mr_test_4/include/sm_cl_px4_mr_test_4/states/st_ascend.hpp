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
      kMissionAltitudeM, kAscendClimbRateMps);
  }

  void runtimeConfigure()
  {
    const float climbSeconds =
      (kMissionAltitudeM - kTakeoffAltitudeM) / kAscendClimbRateMps;
    this->getClientBehavior<OrPx4, CbAscendToAltitude>()->setTimeout(std::chrono::seconds(
      static_cast<long>(climbSeconds * kTimeoutMarginFactor + kTimeoutBaseS)));
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StAscend: climbing to %.1f m", kMissionAltitudeM);
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
