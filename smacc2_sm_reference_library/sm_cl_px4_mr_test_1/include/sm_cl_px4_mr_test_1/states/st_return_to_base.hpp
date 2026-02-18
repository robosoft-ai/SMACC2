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

namespace sm_cl_px4_mr_test_1
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Return to launch position
struct StReturnToBase : smacc2::SmaccState<StReturnToBase, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, MsLanding, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Return to origin (0, 0, -5) NED = home position at 5m altitude
    configure_orthogonal<OrPx4, CbGoToLocation>(0.0f, 0.0f, -5.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StReturnToBase: returning to base...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StReturnToBase: back at base");
  }
};

}  // namespace sm_cl_px4_mr_test_1
