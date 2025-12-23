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
struct StConnected : smacc2::SmaccState<StConnected, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>,
    // Auto-transition to StRelayOn after entry
    Transition<EvCbSuccess<CbRelayOn, OrRelay>, StRelayOn, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // Turn on relay channel 1 as a test
    configure_orthogonal<OrRelay, CbRelayOn>(1);
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StConnected] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StConnected] Connected to Modbus relay!");

    // Get connection info
    cl_modbus_tcp_relay::ClModbusTcpRelay * client;
    this->requiresClient(client);

    if (client && client->getConnectionComponent())
    {
      auto connection = client->getConnectionComponent();
      RCLCPP_INFO(
        getLogger(), "[StConnected] Connection details: %s:%d (slave=%d)",
        connection->getIpAddress().c_str(),
        connection->getPort(),
        connection->getSlaveId());
    }
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StConnected] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
