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
#include <sr_all_events_go/sr_all_events_go.hpp>
#include <sm_husky_barrel_search_1/clients/led_array/client_behaviors.hpp>

namespace sm_husky_barrel_search_1
{
using namespace smacc2::default_events;
using namespace smacc2::state_reactors;
using sm_husky_barrel_search_1::cl_led_array::CbLEDOff;
using sm_husky_barrel_search_1::cl_led_array::CbLEDOn;
using sm_husky_barrel_search_1::cl_led_array::LedColor;

// STATE DECLARATION
struct StAcquireSensors : smacc2::SmaccState<StAcquireSensors, SmHuskyBarrelSearch1>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ON_SENSORS_AVAILABLE : SUCCESS
  {
  };
  struct SrAcquireSensors;

  // TRANSITION TABLE
  typedef mpl::list<

    smacc2::Transition<EvAllGo<SrAllEventsGo, SrAcquireSensors>, StMoveBackwardsBlinking>
    // Transition<EvGlobalError, MsDanceBotRecoveryMode>
    // smacc2::Transition<
    //   smacc2::EvCbSuccess<cl_nav2z::CbWaitPose, OrNavigation>, StNavigateToWaypointX>,

    //smacc2::Transition<smacc2::EvCbSuccess<cl_nav2z::CbWaitNav2Nodes, OrNavigation>, StNavigateToWaypointX, SUCCESS>,

    //smacc2::Transition<
    //  smacc2::EvCbFailure<cl_nav2z::CbWaitPose, OrNavigation>, StNavigateToWaypointX>
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrNavigation, CbLidarSensor>();
    configure_orthogonal<OrNavigation, cl_nav2z::CbWaitPose>();
    configure_orthogonal<OrNavigation, cl_nav2z::CbWaitNav2Nodes>();

    configure_orthogonal<OrLedArray, CbLEDOff>(LedColor::GREEN);
    configure_orthogonal<OrLedArray, CbLEDOff>(LedColor::RED);
    configure_orthogonal<OrLedArray, CbLEDOff>(LedColor::YELLOW);

    // Create State Reactor
    static_createStateReactor<
      SrAllEventsGo, smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrAcquireSensors>,
      mpl::list<
        smacc2::EvCbSuccess<cl_nav2z::CbWaitNav2Nodes, OrNavigation>,
        smacc2::EvCbSuccess<cl_nav2z::CbWaitPose, OrNavigation>
        >>();
  }
};
}  // namespace sm_husky_barrel_search_1
