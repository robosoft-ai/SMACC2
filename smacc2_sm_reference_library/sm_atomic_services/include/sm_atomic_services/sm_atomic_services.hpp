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

#include <smacc2/smacc.hpp>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.hpp>
#include "sm_atomic_services/clients/cl_service_server.hpp"
#include "sm_atomic_services/clients/cl_service_client.hpp"

// ORTHOGONALS
#include <sm_atomic_services/orthogonals/or_timer.hpp>
#include <sm_atomic_services/orthogonals/or_services.hpp>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>
#include "sm_atomic_services/clients/client_behaviors/cb_service_server.hpp"

using namespace boost;
using namespace smacc2;

namespace sm_atomic_services
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicServices
    : public smacc2::SmaccStateMachineBase<SmAtomicServices, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrServices>();
    }
};

} // namespace sm_atomic

#include <sm_atomic_services/states/st_state_1.hpp>
#include <sm_atomic_services/states/st_state_2.hpp>
