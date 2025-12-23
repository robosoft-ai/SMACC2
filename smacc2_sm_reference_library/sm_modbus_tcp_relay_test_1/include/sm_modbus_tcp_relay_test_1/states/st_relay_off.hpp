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

namespace sm_modbus_tcp_relay_test_1
{

using namespace cl_modbus_tcp_relay;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct StRelayOff : smacc2::SmaccState<StRelayOff, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>,
    // On success turning all on, transition to StAllOn
    Transition<EvCbSuccess<CbAllRelaysOn, OrRelay>, StAllOn, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Turn on all relays
    configure_orthogonal<OrRelay, CbAllRelaysOn>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOff] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOff] Relay channel 1 is OFF, now turning ALL ON");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOff] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
