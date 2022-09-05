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
#include <sr_all_events_go/sr_all_events_go.hpp>
#include <sm_husky_barrel_search_1/clients/cb_sleep_for.hpp>
#include <sm_husky_barrel_search_1/clients/led_array/client_behaviors.hpp>


namespace sm_husky_barrel_search_1
{
    using namespace smacc2::default_events;
    using namespace cl_nav2z;
    using namespace smacc2;
    using namespace std::chrono_literals;
    using sm_husky_barrel_search_1::cl_led_array::CbSequenceColorBlinking;

    // STATE DECLARATION
    struct StAirStrikeCommunications : smacc2::SmaccState<StAirStrikeCommunications, SmHuskyBarrelSearch1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
                // Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StEvasionMotion>
                Transition<EvCbFailure<CbAbortNavigation, OrNavigation>, StUndoRetreat>,
                Transition<EvCbSuccess<CbSleepFor, OrNavigation>, StUndoRetreat>
                //smacc2::Transition<EvAllGo<SrAllEventsGo, StAirStrikeCommunications>, StUndoRetreat>



                // Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StUndoRetreat>
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            configure_orthogonal<OrLedArray, CbSequenceColorBlinking>();
            //configure_orthogonal<OrNavigation, CbAbsoluteRotate>(35.0);
            configure_orthogonal<OrNavigation, CbAbortNavigation>();
            configure_orthogonal<OrNavigation, CbSleepFor>(5s);


            // Create State Reactor
            // static_createStateReactor<
            // SrAllEventsGo, smacc2::state_reactors::EvAllGo<SrAllEventsGo, StAirStrikeCommunications>,
            // mpl::list<
            //     smacc2::EvCbSuccess<CbSleepFor, OrNavigation>
            //     // ,
            //     // smacc2::EvCbSuccess<CbAbsoluteRotate, OrNavigation>
            //     // smacc2::EvCbAbort<CbAbortNavigation, OrNavigation>
            //     >>();

            //configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();

        }

        void runtimeConfigure()
        {
        }
    };
} // namespace sm_husky_barrel_search_1
