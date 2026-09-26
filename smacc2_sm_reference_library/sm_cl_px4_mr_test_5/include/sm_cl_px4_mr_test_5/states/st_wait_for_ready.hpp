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

#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: preflight heading gate. PX4 is healthy (StConnectMicroROSAgent) but
// the EKF yaw must also be trustworthy: constant while parked and equal to the
// known spawn heading. Run 19 took off with the heading drifting through 180
// degrees and flipped 4 s after liftoff.
struct StWaitForReady : smacc2::SmaccState<StWaitForReady, MsDisarmedOnGround>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbWaitForHeadingStable, OrPx4>, StArmPX4, SUCCESS>,
    // still drifting after the timeout: keep refusing to arm, re-check
    Transition<EvCbFailure<CbWaitForHeadingStable, OrPx4>, StWaitForReady, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbWaitForHeadingStable>(headingGateParams());
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StWaitForReady: checking the EKF heading before arming...");
  }

  void onExit() {}
};

}  // namespace sm_cl_px4_mr_test_5
