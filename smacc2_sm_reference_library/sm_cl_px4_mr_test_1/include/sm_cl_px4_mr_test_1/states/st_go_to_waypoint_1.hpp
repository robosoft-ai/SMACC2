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

// STATE: Navigate to first waypoint (orbit center)
struct StGoToWaypoint1 : smacc2::SmaccState<StGoToWaypoint1, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbGoToLocation, OrPx4>, StOrbitLocation, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Navigate to waypoint at (10, 0, -5) NED = 10m North, 5m altitude
    configure_orthogonal<OrPx4, CbGoToLocation>(10.0f, 0.0f, -5.0f);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StGoToWaypoint1: navigating to waypoint 1...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StGoToWaypoint1: waypoint 1 reached");
  }
};

}  // namespace sm_cl_px4_mr_test_1
