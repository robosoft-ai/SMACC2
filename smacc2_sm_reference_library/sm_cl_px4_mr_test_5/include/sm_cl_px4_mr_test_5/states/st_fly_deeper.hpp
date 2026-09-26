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

#include <cl_px4_mr/client_behaviors/cb_follow_ned_path.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

#include <algorithm>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: leg B - from the shaft bottom through the portal, down the ramp of the
// 3-way tile and up the far side to the turnaround at x 210. On exit the vertices passed are appended to the
// traversed route so the way back retraces both legs.
struct StFlyDeeper : smacc2::SmaccState<StFlyDeeper, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StHoldAtTurnaround, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbFollowNedPath>(caveRouteBNed(), cruiseFollower());
  }

  void runtimeConfigure()
  {
    const std::vector<NedPoint> route = caveRouteBNed();
    this->getClientBehavior<OrPx4, CbFollowNedPath>()->setTimeout(
      legTimeout(routeLengthM(route) + 20.0f, kCruiseSpeedMps));

    RCLCPP_INFO(
      getLogger(), "StFlyDeeper: leg B, %zu vertices, %.0f m, to NED (%.1f, %.1f, %.1f)",
      route.size(), static_cast<double>(routeLengthM(route)), static_cast<double>(route.back().x),
      static_cast<double>(route.back().y), static_cast<double>(route.back().z));
  }

  void onEntry() {}

  void onExit()
  {
    auto & ms = this->context<MsInFlight>();
    auto * cb = this->getClientBehavior<OrPx4, CbFollowNedPath>();
    const auto & route = cb->path();
    const size_t n = std::min(cb->reachedCount(), route.size());
    ms.traversed.insert(ms.traversed.end(), route.begin(), route.begin() + n);
    RCLCPP_INFO(
      getLogger(), "StFlyDeeper: reached %zu/%zu leg B vertices (%zu traversed in total)", n,
      route.size(), ms.traversed.size());
  }
};

}  // namespace sm_cl_px4_mr_test_5
