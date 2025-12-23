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

#include <modbus/modbus.h>

#include <functional>
#include <mutex>
#include <smacc2/smacc.hpp>
#include <string>

namespace cl_modbus_tcp_relay
{

// Forward declarations for events
template <typename TSource, typename TOrthogonal>
struct EvConnectionLost;

template <typename TSource, typename TOrthogonal>
struct EvConnectionRestored;

/**
 * @brief Component that manages Modbus TCP connection lifecycle and heartbeat monitoring
 *
 * Responsibilities:
 * - Create/destroy modbus_t context
 * - Manage TCP connection state
 * - Periodic heartbeat via ISmaccUpdatable (reads coil status to verify connectivity)
 * - Emit connection state change signals
 * - Thread-safe connection access via mutex
 *
 * Configuration is loaded from YAML parameters:
 *   modbus_relay.ip_address: IP address of the relay (default: "192.168.1.254")
 *   modbus_relay.port: Modbus TCP port (default: 502)
 *   modbus_relay.slave_id: Modbus slave ID (default: 1)
 *   modbus_relay.heartbeat_interval_ms: Heartbeat check interval (default: 1000)
 *   modbus_relay.connect_on_init: Connect automatically on init (default: true)
 */
class CpModbusConnection : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpModbusConnection();
  virtual ~CpModbusConnection();

  void onInitialize() override;

  /**
   * @brief Configure component for event posting during orthogonal allocation
   */
  template <typename TOrthogonal, typename TClient>
  void onStateOrthogonalAllocation()
  {
    postConnectionLostEvent_ = [this]()
    { this->template postEvent<EvConnectionLost<CpModbusConnection, TOrthogonal>>(); };

    postConnectionRestoredEvent_ = [this]()
    { this->template postEvent<EvConnectionRestored<CpModbusConnection, TOrthogonal>>(); };
  }

  // Connection management
  bool connect();
  void disconnect();
  bool reconnect();
  bool isConnected() const;

  // Thread-safe context access
  modbus_t * getContext();
  std::mutex & getMutex();

  // Configuration accessors
  std::string getIpAddress() const { return ip_address_; }
  int getPort() const { return port_; }
  int getSlaveId() const { return slave_id_; }

  // Signals for connection state changes
  smacc2::SmaccSignal<void()> onConnectionLost_;
  smacc2::SmaccSignal<void()> onConnectionRestored_;
  smacc2::SmaccSignal<void(const std::string &)> onConnectionError_;

  // Signal connection helpers
  template <typename T>
  smacc2::SmaccSignalConnection onConnectionLost(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onConnectionLost_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onConnectionRestored(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onConnectionRestored_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onConnectionError(
    void (T::*callback)(const std::string &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onConnectionError_, callback, object);
  }

protected:
  void update() override;  // Heartbeat check

private:
  // Configuration loaded from YAML in onInitialize()
  std::string ip_address_;
  int port_;
  int slave_id_;
  int heartbeat_interval_ms_;
  bool connect_on_init_;

  // Modbus context
  modbus_t * ctx_;
  bool connected_;
  mutable std::mutex mutex_;

  // Event posting functions
  std::function<void()> postConnectionLostEvent_;
  std::function<void()> postConnectionRestoredEvent_;

  // Helper for parameter loading
  template <typename T>
  void declareAndLoadParam(const std::string & name, T & value, const T & default_val);
};

}  // namespace cl_modbus_tcp_relay
