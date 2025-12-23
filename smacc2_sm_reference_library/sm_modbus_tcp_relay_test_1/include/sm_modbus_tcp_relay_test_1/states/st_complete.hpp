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
struct StComplete : smacc2::SmaccState<StComplete, SmModbusTcpRelayTest1>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // If connection is lost, go back to StConnect
    Transition<EvConnectionLost<CpModbusConnection, OrRelay>, StConnect, ABORT>
    > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // No behaviors - final state
  }

  void runtimeConfigure()
  {
    RCLCPP_INFO(getLogger(), "[StComplete] runtimeConfigure()");
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "========================================");
    RCLCPP_INFO(getLogger(), "  cl_modbus_tcp_relay TEST COMPLETE!");
    RCLCPP_INFO(getLogger(), "========================================");
    RCLCPP_INFO(getLogger(), "Test results:");
    RCLCPP_INFO(getLogger(), "  [PASS] CbRelayOn(1) - Single channel ON");
    RCLCPP_INFO(getLogger(), "  [PASS] CbRelayOff(1) - Single channel OFF");
    RCLCPP_INFO(getLogger(), "  [PASS] CbAllRelaysOn - All channels ON");
    RCLCPP_INFO(getLogger(), "  [PASS] CbAllRelaysOff - All channels OFF");
    RCLCPP_INFO(getLogger(), "  [PASS] CbRelayStatus - Read all statuses");
    RCLCPP_INFO(getLogger(), "State machine will remain in StComplete.");
    RCLCPP_INFO(getLogger(), "Press Ctrl-C to exit.");
    RCLCPP_INFO(getLogger(), "========================================");
  }

  void onExit()
  {
    RCLCPP_INFO(getLogger(), "[StComplete] onExit()");
  }
};

}  // namespace sm_modbus_tcp_relay_test_1
