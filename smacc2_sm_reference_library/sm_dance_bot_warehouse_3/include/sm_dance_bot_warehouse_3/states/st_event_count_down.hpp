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
// STATE DECLARATION
struct StEventCountDown : smacc2::SmaccState<StEventCountDown, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCountdownEnd<SrEventCountdown>, StInitialSpinning>,
    Transition<EvGlobalError, MsDanceBotRecoveryMode>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //   configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    //   configure_orthogonal<OrStringPublisher, CbStringPublisher>("Hello World!");
    //   configure_orthogonal<OrTemperatureSensor, CbConditionTemperatureSensor>();
    configure_orthogonal<OrNavigation, CbRotate>(2*M_PI, SpinningPlanner::PureSpinning);

    // Create State Reactor

    //auto srCountdown = static_createStateReactor<SrEventCountdown>(5);
    //srCountdown->addInputEvent<EvTimer<ClRosTimer, OrTimer>>();
    //srCountdown->setOutputEvent<EvCountdownEnd<SrEventCountdown>>();

    auto srCountdown = static_createStateReactor<
      SrEventCountdown, EvCountdownEnd<SrEventCountdown>, mpl::list<EvTimer<ClRosTimer, OrTimer>>>(10);

  }
};
}  // namespace sm_dance_bot_warehouse_3
