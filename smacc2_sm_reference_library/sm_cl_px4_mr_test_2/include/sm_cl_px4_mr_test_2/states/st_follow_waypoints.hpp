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

#include <cmath>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_2
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: Follow a triangle of 3 waypoints at 20m altitude
struct StFollowWaypoints : smacc2::SmaccState<StFollowWaypoints, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowWaypoints, OrPx4>, StHoldPosition5, SUCCESS>
  > reactions;

  static void staticConfigure()
  {
    // Waypoint pattern at 20m altitude (NED: z = -20), going left (negative Y)
    // Each waypoint: {x, y, z, yaw} — yaw=NAN means maintain current heading
    configure_orthogonal<OrPx4, CbFollowWaypoints>(
      std::vector<std::array<float, 4>>{
        {30.0f, 0.0f, -20.0f, NAN},     // 30m North
        {30.0f, -30.0f, -20.0f, NAN},   // 30m North, 30m West
        {0.0f, -30.0f, -20.0f, NAN}     // 30m West
      });
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "StFollowWaypoints: following 3-point triangle...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "StFollowWaypoints: all waypoints reached");
  }
};

}  // namespace sm_cl_px4_mr_test_2
