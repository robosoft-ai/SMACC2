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
 *****************************************************************************************************************/

#include <smacc2/smacc.hpp>

namespace sm_atomic_mode_states
{
// STATE DECLARATION
struct State2 : smacc2::SmaccState<State2, MsState2>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsState1, SUCCESS>

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(2); // EvTimer triggers once at 10 client ticks
        configure_orthogonal<OrTimer, CbUpdatableTest>();
    }

    void runtimeConfigure()
    {
        RCLCPP_INFO(getLogger(),"Entering State2");
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(),"On Entry!");
    }

    void onExit()
    {
        RCLCPP_INFO(getLogger(),"On Exit!");
    }

};
}
