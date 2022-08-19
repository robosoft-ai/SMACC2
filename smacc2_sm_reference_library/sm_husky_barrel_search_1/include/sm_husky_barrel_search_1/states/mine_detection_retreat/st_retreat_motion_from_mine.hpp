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
#include <nav2z_client/nav2z_client.hpp>
#include <nav2z_client/client_behaviors.hpp>
#include <sm_husky_barrel_search_1/clients/cb_sleep_for.hpp>
#include <sm_husky_barrel_search_1/clients/led_array/client_behaviors.hpp>

namespace sm_husky_barrel_search_1
{
using cl_nav2z::CbNavigateBackwards;

// STATE DECLARATION
struct StRetreatMotionFromMine : smacc2::SmaccState<StRetreatMotionFromMine, SmHuskyBarrelSearch1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<Transition<EvCbSuccess<CbSequence, OrNavigation>, StSelectSaferMinePath, SUCCESS>,
                    Transition<EvCbFailure<CbSequence, OrNavigation>, StRetreatMotionFromMine, ABORT>>
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbSequence>();
    configure_orthogonal<OrLedArray, CbBlinking>(LedColor::YELLOW);
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrNavigation, CbSequence>()
              ->then<OrNavigation, CbNavigateBackwards>(1.5)
              ->then<OrNavigation, CbRotate>(-60.0)
              ->then<OrNavigation, CbNavigateForward>(4.0);
  }
};
}  // namespace sm_husky_barrel_search_1
