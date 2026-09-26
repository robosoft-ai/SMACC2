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

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: fly the traversed route back to the spawn point at cruise altitude
struct StFlyInbound : smacc2::SmaccState<StFlyInbound, SsCaveMission>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StFlyToBaseStation, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, StReturnHome, ABORT>
  > reactions;

  static void staticConfigure()
  {
    configure_orthogonal<OrPx4, CbFollowNedPath>(std::vector<NedPoint>{}, cruiseFollower());
  }

  void runtimeConfigure()
  {
    auto & ms = this->context<MsInFlight>();
    auto * cb = this->getClientBehavior<OrPx4, CbFollowNedPath>();
    const std::vector<NedPoint> path = retracePath(ms.traversed);
    cb->setPath(path);
    const float lengthM = routeLengthM(path) + 20.0f;
    cb->setTimeout(legTimeout(lengthM, kCruiseSpeedMps));

    RCLCPP_INFO(
      getLogger(), "StFlyInbound: retracing %zu vertices home, %.0f m", ms.traversed.size(),
      static_cast<double>(routeLengthM(path)));
  }

  void onEntry() {}

  void onExit()
  {
    auto & ms = this->context<MsInFlight>();
    auto * cb = this->getClientBehavior<OrPx4, CbFollowNedPath>();
    ms.retraceConsumed(cb->reachedCount());
    RCLCPP_INFO(
      getLogger(), "StFlyInbound: %zu route vertices still outbound of the vehicle",
      ms.traversed.size());
  }
};

}  // namespace sm_cl_px4_mr_test_5
