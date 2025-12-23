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
struct StAllOn : smacc2::SmaccState<StAllOn, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>,
    // On success turning all off, transition to StAllOff
    Transition<EvCbSuccess<CbAllRelaysOff, OrRelay>, StAllOff, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Turn off all relays
    configure_orthogonal<OrRelay, CbAllRelaysOff>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StAllOn] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StAllOn] All relays are ON, now turning ALL OFF");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StAllOn] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
