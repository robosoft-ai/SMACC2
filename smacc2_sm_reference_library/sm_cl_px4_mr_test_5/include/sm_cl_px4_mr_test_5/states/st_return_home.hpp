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
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

using namespace cl_px4_mr;
using namespace smacc2::default_transition_tags;

// STATE: mission abort leg - retrace the traversed route back to the spawn
// point (a straight line home would fly into the cave walls), then land
// whatever happens. An obstacle on the way back stops the vehicle again.
struct StReturnHome : smacc2::SmaccState<StReturnHome, MsInFlight>
{
  using SmaccState::SmaccState;

  typedef mpl::list<
    // back over the pad: the demo still ends with the base-station finale
    Transition<EvCbSuccess<CbFollowNedPath, OrPx4>, StFlyToBaseStation, SUCCESS>,
    Transition<EvCbFailure<CbFollowNedPath, OrPx4>, MsLanding, ABORT>,
    Transition<EvObstacleTooClose<CbObstacleGuard, OrLidar>, StObstacleHold, ABORT>
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
    cb->setTimeout(legTimeout(routeLengthM(path) + 20.0f, kCruiseSpeedMps));
  }

  void onEntry()
  {
    RCLCPP_WARN(
      getLogger(), "StReturnHome: CAVE LEG ABORTED - retracing %zu vertices home, then the finale",
      this->context<MsInFlight>().traversed.size());
  }

  void onExit()
  {
    auto & ms = this->context<MsInFlight>();
    auto * cb = this->getClientBehavior<OrPx4, CbFollowNedPath>();
    ms.retraceConsumed(cb->reachedCount());
  }
};

}  // namespace sm_cl_px4_mr_test_5
