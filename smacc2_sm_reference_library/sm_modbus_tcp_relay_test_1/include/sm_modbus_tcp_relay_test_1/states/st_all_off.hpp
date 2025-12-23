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
struct StAllOff : smacc2::SmaccState<StAllOff, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>,
    // After reading status, go to StComplete
    Transition<EvCbSuccess<CbRelayStatus, OrRelay>, StComplete, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Read all relay statuses
    configure_orthogonal<OrRelay, CbRelayStatus>();
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StAllOff] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StAllOff] All relays are OFF, reading status...");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StAllOff] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
