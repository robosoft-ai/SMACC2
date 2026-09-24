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

#include <cl_px4_mr/client_behaviors/cb_lawnmower.hpp>
#include <config/mission_constants.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_lawnmower_2.hpp>

namespace sm_cl_px4_mr_test_4
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// INNER STATE: fly the pattern at the pin, then on to StTransitToGridPattern1
struct StiLawnmower2Run : smacc2::SmaccState<StiLawnmower2Run, SsLawnmower2>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbLawnmower, OrPx4>, StTransitToGridPattern1, SUCCESS>,
    Transition<EvCbFailure<CbLawnmower, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbLawnmower>();
  }

  void runtimeConfigure()
  {
    const auto p = SsLawnmower2::patternParams();
    auto * cb = this->getClientBehavior<OrPx4, CbLawnmower>();
    cb->setParams(p);
    cb->setFollowerParams(SsLawnmower2::followerParams());
    cb->setTimeout(patternTimeout(flightPatternLawnmowerLength(p), kPatternSpeedMps));

    const NedXY pin = SsLawnmower2::pin();
    RCLCPP_INFO(
      getLogger(), "StiLawnmower2Run: pin P4 NED (%.1f, %.1f)", static_cast<double>(pin.x),
      static_cast<double>(pin.y));
  }

  void onEntry() {}
  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_4
