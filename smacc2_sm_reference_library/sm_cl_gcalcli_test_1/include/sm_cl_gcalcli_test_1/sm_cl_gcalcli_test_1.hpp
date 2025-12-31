// Copyright 2024 RobosoftAI Inc.
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

#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_ros2_timer/cl_ros2_timer.hpp>

#include <cl_gcalcli/cl_gcalcli.hpp>
#include <cl_gcalcli/client_behaviors.hpp>
#include <cl_gcalcli/components/cp_calendar_event_listener.hpp>
#include <cl_gcalcli/components/cp_calendar_poller.hpp>
#include <cl_gcalcli/components/cp_gcalcli_connection.hpp>
#include <cl_gcalcli/events.hpp>

// CLIENT BEHAVIORS
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_loop.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>

// ORTHOGONALS
#include "orthogonals/or_calendar.hpp"
#include "orthogonals/or_timer.hpp"

using namespace boost;
using namespace smacc2;
using namespace cl_ros2_timer;
using namespace cl_gcalcli;

namespace sm_cl_gcalcli_test_1
{

// Forward declare states
class StInit;
class StWaitConnection;
class StTestRefresh;
class StTestEventDetect;
class StTestQuickAdd;
class StDone;

//--------------------------------------------------------------------
// STATE_MACHINE
struct SmClGcalcliTest1 : public smacc2::SmaccStateMachineBase<SmClGcalcliTest1, StInit>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrCalendar>();
  }
};

}  // namespace sm_cl_gcalcli_test_1

// Include states after state machine declaration
#include "states/st_init.hpp"
#include "states/st_wait_connection.hpp"
#include "states/st_test_refresh.hpp"
#include "states/st_test_event_detect.hpp"
#include "states/st_test_quick_add.hpp"
#include "states/st_done.hpp"
