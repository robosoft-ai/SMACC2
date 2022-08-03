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
    using namespace smacc2::default_events;
    using namespace cl_nav2z;
    using namespace smacc2;
    using namespace std::chrono_literals;

    // STATE DECLARATION
    struct StNavigateBarriersForwardNext : smacc2::SmaccState<StNavigateBarriersForwardNext, SmHuskyBarrelSearch1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
              Transition<EvGoalWaypointReached<CbNavigateNextWaypointUntilReached, OrNavigation>, StRetreatMotionFromMine, SUCCESS>,
              Transition<EvCbSuccess<CbNavigateNextWaypointUntilReached, OrNavigation>, StNavigateBarriersForwardNext, SUCCESS>,
              Transition<EvCbFailure<CbNavigateNextWaypointUntilReached, OrNavigation>, StNavigateBarriersForwardNext, ABORT>
              //Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StUndoRetreat>
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            // configure_orthogonal<OrNavigation, CbSleepFor>(10s);
            configure_orthogonal<OrNavigation, CbNavigateNextWaypointUntilReached>("pre-mine-collision",
                                                                                NavigateNextWaypointOptions
                                                                                {
                                                                                    .controllerName_="SuperFastPathFollow",
                                                                                    .goalCheckerName_ = "super_fast_follow_path_goal_checker"
                                                                                });
        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_husky_barrel_search_1
