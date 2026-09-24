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

#include <cl_px4_mr/client_behaviors/cb_square_spiral.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_square_spiral_2.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// INNER STATE: fly the pattern at the pin, then on to StTransitToLawnmower1
struct StiSquareSpiral2Run : smacc2::SmaccState<StiSquareSpiral2Run, SsSquareSpiral2>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbSquareSpiral, OrPx4>, StTransitToLawnmower1, SUCCESS>,
    Transition<EvCbFailure<CbSquareSpiral, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbSquareSpiral>();
  }

  void runtimeConfigure()
  {
    const auto p = SsSquareSpiral2::patternParams();
    auto * cb = this->getClientBehavior<OrPx4, CbSquareSpiral>();
    cb->setParams(p);
    cb->setFollowerParams(SsSquareSpiral2::followerParams());
    cb->setTimeout(patternTimeout(flightPatternSquareSpiralLength(p), kPatternSpeedMps));

    const NedXY pin = SsSquareSpiral2::pin();
    RCLCPP_INFO(
      getLogger(), "StiSquareSpiral2Run: pin P2 NED (%.1f, %.1f)", static_cast<double>(pin.x),
      static_cast<double>(pin.y));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
