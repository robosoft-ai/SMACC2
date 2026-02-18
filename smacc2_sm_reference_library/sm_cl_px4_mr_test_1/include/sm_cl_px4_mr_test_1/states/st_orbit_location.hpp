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

// STATE: Orbit around waypoint 1
struct StOrbitLocation : smacc2::SmaccState<StOrbitLocation, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbOrbitLocation, OrPx4>, StReturnToBase, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Orbit center (10, 0) at 5m altitude, 5m radius, 0.5 rad/s, 3 orbits
    configure_orthogonal<OrPx4, CbOrbitLocation>(10.0f, 0.0f, 5.0f, 5.0f, 0.5f, 3);
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StOrbitLocation: beginning orbit...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StOrbitLocation: orbit complete");
  }
};

}  // namespace sm_cl_px4_mr_test_1
