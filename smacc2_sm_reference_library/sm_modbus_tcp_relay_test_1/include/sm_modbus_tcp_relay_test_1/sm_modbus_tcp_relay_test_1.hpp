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
#include <cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp>

// CLIENT BEHAVIORS
#include <cl_modbus_tcp_relay/client_behaviors/cb_relay_on.hpp>
#include <cl_modbus_tcp_relay/client_behaviors/cb_relay_off.hpp>
#include <cl_modbus_tcp_relay/client_behaviors/cb_all_relays_on.hpp>
#include <cl_modbus_tcp_relay/client_behaviors/cb_all_relays_off.hpp>
#include <cl_modbus_tcp_relay/client_behaviors/cb_relay_status.hpp>

// ORTHOGONALS
#include "orthogonals/or_relay.hpp"

using namespace boost;
using namespace smacc2;

namespace sm_modbus_tcp_relay_test_1
{

// Forward declarations for states
class StConnect;
class StConnected;
class StRelayOn;
class StRelayOff;
class StAllOn;
class StAllOff;
class StReadStatus;
class StComplete;

//--------------------------------------------------------------------
// STATE MACHINE
struct SmModbusTcpRelayTest1
  : public smacc2::SmaccStateMachineBase<SmModbusTcpRelayTest1, StConnect>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrRelay>();
  }
};

}  // namespace sm_modbus_tcp_relay_test_1

#include "states/st_connect.hpp"
#include "states/st_connected.hpp"
#include "states/st_relay_on.hpp"
#include "states/st_relay_off.hpp"
#include "states/st_all_on.hpp"
#include "states/st_all_off.hpp"
#include "states/st_read_status.hpp"
#include "states/st_complete.hpp"
