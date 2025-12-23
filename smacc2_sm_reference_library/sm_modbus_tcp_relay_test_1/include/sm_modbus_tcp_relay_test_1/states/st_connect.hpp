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
struct StConnect : smacc2::SmaccState<StConnect, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // When connection is restored, transition to StConnected
    Transition<EvConnectionRestored<CpModbusConnection, OrRelay>, StConnected, SUCCESS>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // No client behaviors in this initial state
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StConnect] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StConnect] Waiting for Modbus connection...");

    // Check if already connected (connect_on_init may have succeeded)
    cl_modbus_tcp_relay::ClModbusTcpRelay * client;
    this->requiresClient(client);

    if (client && client->getConnectionComponent())
    {
      auto connection = client->getConnectionComponent();
      if (connection->isConnected())
      {
        RCLCPP_INFO(getLogger(), "[StConnect] Already connected! Posting restored event.");
        this->postEvent<EvConnectionRestored<CpModbusConnection, OrRelay>>();
      }
    }
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StConnect] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
