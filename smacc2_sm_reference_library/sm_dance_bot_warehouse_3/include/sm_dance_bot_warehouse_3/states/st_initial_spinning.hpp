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

#include <smacc2/smacc.hpp>

namespace sm_dance_bot_warehouse_3
{
using cl_nav2zclient::CbPureSpinning;

// STATE DECLARATION
struct StInitialSpinning : smacc2::SmaccState<StInitialSpinning, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbPureSpinning, OrNavigation>, StNavigateToWaypoint1, SUCCESS>,
    Transition<EvCbFailure<CbPureSpinning, OrNavigation>, StNavigateToWaypoint1, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbPureSpinning>(2.0*M_PI, 1.0 /*rad_s*/);
    configure_orthogonal<OrNavigation, CbResumeSlam>();
  }
};
}  // namespace sm_dance_bot_warehouse_3
