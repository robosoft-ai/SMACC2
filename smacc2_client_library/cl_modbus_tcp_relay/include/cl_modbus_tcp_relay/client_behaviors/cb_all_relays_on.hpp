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

#pragma once

#include <cl_modbus_tcp_relay/cl_modbus_tcp_relay.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_modbus_tcp_relay
{

/**
 * @brief Client behavior to turn ON all relay channels simultaneously
 *
 * Posts EvCbSuccess on successful write, EvCbFailure on error.
 */
class CbAllRelaysOn : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbAllRelaysOn() {}

  virtual ~CbAllRelaysOn() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresClient(client_);
    this->requiresComponent(relayComponent_);
  }

  virtual void onEntry() override
  {
    RCLCPP_INFO(getLogger(), "[CbAllRelaysOn] Turning ON all channels");

    if (!relayComponent_)
    {
      RCLCPP_ERROR(getLogger(), "[CbAllRelaysOn] Relay component not available");
      this->postFailureEvent();
      return;
    }

    bool success = relayComponent_->writeAllCoils(true);

    if (success)
    {
      RCLCPP_INFO(getLogger(), "[CbAllRelaysOn] All channels turned ON successfully");
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbAllRelaysOn] Failed to turn ON all channels");
      this->postFailureEvent();
    }
  }

private:
  ClModbusTcpRelay * client_;
  CpModbusRelay * relayComponent_;
};

}  // namespace cl_modbus_tcp_relay
