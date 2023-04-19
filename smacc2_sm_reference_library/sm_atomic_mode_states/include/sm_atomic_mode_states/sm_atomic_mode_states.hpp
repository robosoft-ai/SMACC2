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

// CLIENTS
#include <ros_timer_client/cl_ros_timer.hpp>

// ORTHOGONALS
#include <sm_atomic_mode_states/orthogonals/or_timer.hpp>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>
#include <sm_atomic_mode_states/client_behaviors/cb_updatable_test.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_atomic_mode_states
{

//STATE
class State1;
class State2;
class MsState1;
class MsState2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicModeStates
    : public smacc2::SmaccStateMachineBase<SmAtomicModeStates, MsState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
    }
};

} // namespace sm_atomic_mode_states

#include <sm_atomic_mode_states/states/ms_state_1.hpp>
#include <sm_atomic_mode_states/states/ms_state_2.hpp>

#include <sm_atomic_mode_states/states/st_state_1.hpp>
#include <sm_atomic_mode_states/states/st_state_2.hpp>
