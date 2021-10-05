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

#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>
#include <smacc2/smacc.hpp>

namespace sm_dance_bot_lite
{
// STATE DECLARATION
struct StNavigateForward1 : smacc2::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees2>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX, ABORT>
    //, Transition<EvActionPreempted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX, PREEMPT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure()
  {
    ClMoveBaseZ * move_base_action_client;
    this->requiresClient(move_base_action_client);

    // we careful with the lifetime of the callbac, us a scoped connection if is not forever
    move_base_action_client->onSucceeded(&StNavigateForward1::onActionClientSucceeded, this);
  }

  void onActionClientSucceeded(cl_move_base_z::ClMoveBaseZ::WrappedResult & msg)
  {
    RCLCPP_INFO_STREAM(
      getLogger(),
      " [Callback SmaccSignal] Success Detected from StAquireSensors (connected to client signal), "
      "result data: "
        << msg.result);
  }
};
}  // namespace sm_dance_bot_lite
