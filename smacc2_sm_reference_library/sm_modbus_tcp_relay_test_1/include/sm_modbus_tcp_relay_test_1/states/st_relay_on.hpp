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
struct StRelayOn : smacc2::SmaccState<StRelayOn, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>,
    // On success, transition to StRelayOff
    Transition<EvCbSuccess<CbRelayOff, OrRelay>, StRelayOff, SUCCESS>,
    // On failure, stay here (could add error handling state)
    Transition<EvCbFailure<CbRelayOff, OrRelay>, StConnect, ABORT>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Turn off relay channel 1 (toggle test)
    configure_orthogonal<OrRelay, CbRelayOff>(1);
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOn] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOn] Relay channel 1 is ON, will turn OFF");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StRelayOn] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
