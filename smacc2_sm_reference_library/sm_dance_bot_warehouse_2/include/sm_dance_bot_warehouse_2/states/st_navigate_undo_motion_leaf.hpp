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

namespace sm_dance_bot_warehouse_2
{
  using ::cl_nav2z::ClNav2Z;

// STATE DECLARATION
struct StNavigateUndoMotionLeaf : smacc2::SmaccState<StNavigateUndoMotionLeaf, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StNavigateUndoMotion, SUCCESS>,
    Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StNavigateUndoMotionLeaf, ABORT>

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

  void onExit(ABORT)
  {

  }

};
}  // namespace sm_dance_bot_warehouse_2
