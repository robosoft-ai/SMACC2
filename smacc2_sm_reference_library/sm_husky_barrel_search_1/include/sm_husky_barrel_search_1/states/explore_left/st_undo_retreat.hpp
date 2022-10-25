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
#include <smacc2/client_behaviors/cb_sequence.hpp>
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
    using sm_husky_barrel_search_1::cl_led_array::CbBlinking;
    using smacc2::client_behaviors::CbSequence;

    // STATE DECLARATION
    struct StUndoRetreat : smacc2::SmaccState<StUndoRetreat, SmHuskyBarrelSearch1>
    {
        using SmaccState::SmaccState;

        // TRANSITION TABLE
        typedef mpl::list<
                //Transition<EvCbSuccess<CbSequence, OrNavigation>, StEvasionMotion>,

                // undo testing
                Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StEvasionMotion>,
                Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StUndoRetreat>

                // backward motion
                // Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StEvasionMotion>,
                // Transition<EvCbFailure<CbNavigateNextWaypoint, OrNavigation>, StUndoRetreat>
            >
            reactions;

        // STATE FUNCTIONS
        static void staticConfigure()
        {
            // configure_orthogonal<OrNavigation, CbNavigateBackwards>(8);
            configure_orthogonal<OrNavigation, CbUndoPathBackwards>();

            // configure_orthogonal<OrNavigation, CbSequence>();
            configure_orthogonal<OrLedArray, CbBlinking>(LedColor::YELLOW);
            //configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
        }

        void runtimeConfigure()
        {
            //     auto cbSequence =  this->getClientBehavior<OrNavigation, CbSequence>();

            //     cbSequence->then<OrNavigation,CbNavigateNextWaypointUntilReached>("base-entrance")
            //     ->then<OrNavigation,CbSleepFor>(6s)
            //     ->then<OrNavigation,CbUndoPathBackwards>(
            //         CbUndoPathBackwardsOptions{
            //         .undoControllerName_ = "UndoBackwardLocalPlanner"
            //         }
            //     ->then<OrNavigation,CbUndoPathBackwards>(
            //         CbUndoPathBackwardsOptions{
            //         .undoControllerName_ = "UndoBackwardLocalPlanner"
            //         }
            //   );

            // auto cbsequence = this->template getClientBehavior<OrNavigation, CbSequence>();
            // cbsequence
            // ->then<OrNavigation, CbUndoPathBackwards>()
            // ->then<OrNavigation, CbUndoPathBackwards>();
        }
    };
} // namespace sm_husky_barrel_search_1
