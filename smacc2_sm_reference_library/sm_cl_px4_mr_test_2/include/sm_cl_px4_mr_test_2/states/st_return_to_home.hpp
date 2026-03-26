// Copyright 2025 Robosoft Inc.
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

namespace sm_cl_px4_mr_test_2
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Return to home position (origin at 15m altitude)
struct StReturnToHome : smacc2::SmaccState<StReturnToHome, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbReturnToHome, OrPx4>, MsLanding, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Return to origin at 15m altitude (NED: z = -15), heading North (yaw = 0)
    configure_orthogonal<OrPx4, CbReturnToHome>(0.0f, 0.0f, -15.0f, 0.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StReturnToHome: returning to home position...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StReturnToHome: home position reached");
  }
};

}  // namespace sm_cl_px4_mr_test_2
