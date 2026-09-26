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

#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

#include <algorithm>
#include <vector>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;

// MODE STATE: airborne. Hosts the one CbObstacleGuard for the whole flight
// (container-state behavior: one instance, one event per detection, survives
// every inner transition) and the data the abort legs share.
//
// The nominal legs live in SsCaveMission; StObstacleHold and StReturnHome are
// its siblings here. Reactions must NOT be declared on this state with an
// inner target: Boost.Statechart transitions are external, so that would exit
// and re-enter MsInFlight, destroying the guard and this data.
struct MsInFlight : smacc2::SmaccState<MsInFlight, SmClPx4MrTest5, SsCaveMission>
{
  using SmaccState::SmaccState;

  // route vertices the vehicle has actually flown past on the way out (cruise
  // altitude); an abort retraces them in reverse. Written by StFlyOutbound,
  // consumed by StFlyInbound / StReturnHome.
  std::vector<NedPoint> traversed;
  int obstacleHolds = 0;

  // a retrace flew k of the traversed vertices backwards: drop them
  void retraceConsumed(size_t k)
  {
    traversed.resize(traversed.size() - std::min(k, traversed.size()));
  }

  typedef mpl::list<
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrLidar, CbObstacleGuard>();
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "--- MsInFlight ---");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "--- Exiting MsInFlight ---");
  }
};

}  // namespace sm_cl_px4_mr_test_5
