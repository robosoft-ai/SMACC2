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

namespace sm_dance_bot_warehouse_3
{
  using ::cl_nav2z::ClNav2Z;
  using namespace std::chrono_literals;

// STATE DECLARATION
struct StNavigateUndoMotion : smacc2::SmaccState<StNavigateUndoMotion, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StNavigateUndoMotionLeaf, SUCCESS>,
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StNavigateUndoMotion, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
    configure_orthogonal<OrNavigation, CbPauseSlam>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {

  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "Waiting to leave space to the undo planner");
    rclcpp::sleep_for(10s);
  }

};
}  // namespace sm_dance_bot_warehouse_3
