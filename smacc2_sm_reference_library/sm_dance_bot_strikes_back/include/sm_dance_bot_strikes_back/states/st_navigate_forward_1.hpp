// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc.hpp>

namespace sm_dance_bot_strikes_back
{
// STATE DECLARATION
struct StNavigateForward1 : smacc2::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees2>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateForward1, ABORT>
    //, Transition<EvActionPreempted<ClNav2Z, OrNavigation>, StNavigateToWaypointsX, PREEMPT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();

  }

  void runtimeConfigure()
  {
    ClNav2Z * move_base_action_client;
    this->requiresClient(move_base_action_client);

    // we careful with the lifetime of the callbac, us a scoped connection if is not forever
    move_base_action_client->onSucceeded(&StNavigateForward1::onActionClientSucceeded, this);
  }

  void onActionClientSucceeded(cl_nav2z::ClNav2Z::WrappedResult & msg)
  {
    RCLCPP_INFO_STREAM(
      getLogger(),
      " [Callback SmaccSignal] Success Detected from StAcquireSensors (connected to client signal), "
      "result data: "
        << msg.result);
  }
};
}  // namespace sm_dance_bot_strikes_back
