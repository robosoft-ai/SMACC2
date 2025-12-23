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

#include <cl_modbus_tcp_relay/components/cp_modbus_connection.hpp>
#include <cl_modbus_tcp_relay/components/cp_modbus_relay.hpp>
#include <smacc2/smacc.hpp>

namespace cl_modbus_tcp_relay
{

// Connection events
template <typename TSource, typename TOrthogonal>
struct EvConnectionLost : sc::event<EvConnectionLost<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvConnectionRestored : sc::event<EvConnectionRestored<TSource, TOrthogonal>>
{
};

// Relay operation events
template <typename TSource, typename TOrthogonal>
struct EvRelayWriteSuccess : sc::event<EvRelayWriteSuccess<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvRelayWriteFailure : sc::event<EvRelayWriteFailure<TSource, TOrthogonal>>
{
};

/**
 * @brief SMACC2 Client for controlling Modbus TCP relays
 *
 * This client manages connection and control of Modbus TCP relay devices,
 * specifically designed for 8-channel relays like the Waveshare POE ETH Relay.
 *
 * Configuration is loaded from YAML parameters:
 *   modbus_relay.ip_address: IP address of the relay (default: "192.168.1.254")
 *   modbus_relay.port: Modbus TCP port (default: 502)
 *   modbus_relay.slave_id: Modbus slave ID (default: 1)
 *   modbus_relay.heartbeat_interval_ms: Heartbeat check interval (default: 1000)
 *   modbus_relay.connect_on_init: Connect automatically on init (default: true)
 */
class ClModbusTcpRelay : public smacc2::ISmaccClient
{
public:
  ClModbusTcpRelay();

  virtual ~ClModbusTcpRelay();

  void onInitialize() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // Create connection component if not already created
    if (!connectionComponent_)
    {
      connectionComponent_ =
        this->template createComponent<CpModbusConnection, TOrthogonal, ClModbusTcpRelay>();
    }

    // Create relay component if not already created
    if (!relayComponent_)
    {
      relayComponent_ =
        this->template createComponent<CpModbusRelay, TOrthogonal, ClModbusTcpRelay>();
    }

    // Configure components for this orthogonal
    connectionComponent_->template onStateOrthogonalAllocation<TOrthogonal, ClModbusTcpRelay>();
    relayComponent_->template onStateOrthogonalAllocation<TOrthogonal, ClModbusTcpRelay>();
  }

  // Accessor for connection component
  CpModbusConnection * getConnectionComponent() { return connectionComponent_; }

  // Accessor for relay component
  CpModbusRelay * getRelayComponent() { return relayComponent_; }

private:
  CpModbusConnection * connectionComponent_ = nullptr;
  CpModbusRelay * relayComponent_ = nullptr;
};

}  // namespace cl_modbus_tcp_relay
