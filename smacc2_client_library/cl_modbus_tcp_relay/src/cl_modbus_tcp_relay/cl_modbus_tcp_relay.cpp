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

/*****************************************************************************************************************
 *
 *   Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp>

namespace cl_modbus_tcp_relay
{

ClModbusTcpRelay::ClModbusTcpRelay() {}

ClModbusTcpRelay::~ClModbusTcpRelay() {}

void ClModbusTcpRelay::onInitialize()
{
  // Components will be created here in Phase 2
  RCLCPP_INFO(getLogger(), "[ClModbusTcpRelay] Client initialized");
}

}  // namespace cl_modbus_tcp_relay
