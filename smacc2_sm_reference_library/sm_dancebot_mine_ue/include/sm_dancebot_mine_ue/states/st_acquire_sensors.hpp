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

namespace sm_dancebot_mine_ue
{
using namespace smacc2::default_events;
using smacc2::client_behaviors::CbSleepFor;
using namespace std::chrono_literals;

// STATE DECLARATION
struct StAcquireSensors : smacc2::SmaccState<StAcquireSensors, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ON_SENSORS_AVAILABLE : SUCCESS{};
  struct SrAcquireSensors;

  // TRANSITION TABLE
  typedef mpl::list<

    // 
    Transition<EvAllGo<SrAllEventsGo, SrAcquireSensors>, StNavigateMineWaypointsX, ON_SENSORS_AVAILABLE>,
    Transition<EvGlobalError, MsDanceBotRecoveryMode>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, sm_dancebot_mine_ue::CbLoadWaypointsFile>("waypoints_mine", "sm_dancebot_mine_ue");
    configure_orthogonal<OrNavigation,CbSleepFor>(5s);


    // Create State Reactor
    auto srAllSensorsReady = static_createStateReactor<
      SrAllEventsGo, smacc2::state_reactors::EvAllGo<SrAllEventsGo, SrAcquireSensors>,
      mpl::list<
        // EvTopicMessage<CbConditionTemperatureSensor, OrTemperatureSensor>,
        // EvCbSuccess<CbWaitPose, OrNavigation>,
        EvCbSuccess<CbSleepFor, OrNavigation>
        >>();
  }
};
}  // namespace sm_dancebot_mine_ue
