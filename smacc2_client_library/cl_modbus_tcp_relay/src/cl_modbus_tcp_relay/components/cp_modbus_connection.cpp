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

#include <cl_modbus_tcp_relay/components/cp_modbus_connection.hpp>

namespace cl_modbus_tcp_relay
{

CpModbusConnection::CpModbusConnection()
: ISmaccUpdatable(rclcpp::Duration::from_seconds(1.0)),  // Default 1 second update period
  ip_address_("192.168.1.254"),
  port_(502),
  slave_id_(1),
  heartbeat_interval_ms_(1000),
  connect_on_init_(true),
  ctx_(nullptr),
  connected_(false)
{
}

CpModbusConnection::~CpModbusConnection()
{
  disconnect();
  if (ctx_)
  {
    modbus_free(ctx_);
    ctx_ = nullptr;
  }
}

template <typename T>
void CpModbusConnection::declareAndLoadParam(
  const std::string & name, T & value, const T & default_val)
{
  auto node = getNode();
  if (!node->has_parameter(name))
  {
    node->declare_parameter(name, default_val);
  }
  node->get_parameter(name, value);
}

// Explicit specialization for string type
template <>
void CpModbusConnection::declareAndLoadParam<std::string>(
  const std::string & name, std::string & value, const std::string & default_val)
{
  auto node = getNode();
  if (!node->has_parameter(name))
  {
    node->declare_parameter(name, default_val);
  }
  node->get_parameter(name, value);
  RCLCPP_INFO(getLogger(), "[CpModbusConnection] %s: %s", name.c_str(), value.c_str());
}

void CpModbusConnection::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpModbusConnection] Initializing...");

  // Load configuration from YAML with defaults
  declareAndLoadParam("modbus_relay.ip_address", ip_address_, std::string("192.168.1.254"));
  declareAndLoadParam("modbus_relay.port", port_, 502);
  declareAndLoadParam("modbus_relay.slave_id", slave_id_, 1);
  declareAndLoadParam("modbus_relay.heartbeat_interval_ms", heartbeat_interval_ms_, 1000);
  declareAndLoadParam("modbus_relay.connect_on_init", connect_on_init_, true);

  RCLCPP_INFO(
    getLogger(),
    "[CpModbusConnection] Config: %s:%d (slave=%d, heartbeat=%dms, connect_on_init=%s)",
    ip_address_.c_str(), port_, slave_id_, heartbeat_interval_ms_,
    connect_on_init_ ? "true" : "false");

  // Set update period for heartbeat
  this->setUpdatePeriod(rclcpp::Duration::from_seconds(heartbeat_interval_ms_ / 1000.0));

  // Create modbus context
  ctx_ = modbus_new_tcp(ip_address_.c_str(), port_);
  if (ctx_)
  {
    modbus_set_slave(ctx_, slave_id_);
    modbus_set_response_timeout(ctx_, 1, 0);  // 1 second timeout

    if (connect_on_init_)
    {
      connect();
    }
  }
  else
  {
    std::string error_msg = "Failed to create modbus context";
    RCLCPP_ERROR(getLogger(), "[CpModbusConnection] %s", error_msg.c_str());
    onConnectionError_(error_msg);
  }
}

bool CpModbusConnection::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ctx_)
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusConnection] Cannot connect: context is null");
    return false;
  }

  if (connected_)
  {
    RCLCPP_DEBUG(getLogger(), "[CpModbusConnection] Already connected");
    return true;
  }

  RCLCPP_INFO(
    getLogger(), "[CpModbusConnection] Connecting to %s:%d...", ip_address_.c_str(), port_);

  int rc = modbus_connect(ctx_);
  if (rc == -1)
  {
    std::string error_msg = std::string("Connection failed: ") + modbus_strerror(errno);
    RCLCPP_ERROR(getLogger(), "[CpModbusConnection] %s", error_msg.c_str());
    onConnectionError_(error_msg);
    return false;
  }

  connected_ = true;
  RCLCPP_INFO(getLogger(), "[CpModbusConnection] Connected successfully");
  return true;
}

void CpModbusConnection::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (ctx_ && connected_)
  {
    modbus_close(ctx_);
    connected_ = false;
    RCLCPP_INFO(getLogger(), "[CpModbusConnection] Disconnected");
  }
}

bool CpModbusConnection::reconnect()
{
  RCLCPP_INFO(getLogger(), "[CpModbusConnection] Reconnecting...");
  disconnect();
  return connect();
}

bool CpModbusConnection::isConnected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

modbus_t * CpModbusConnection::getContext() { return ctx_; }

std::mutex & CpModbusConnection::getMutex() { return mutex_; }

void CpModbusConnection::update()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!ctx_)
  {
    return;
  }

  if (!connected_)
  {
    // Try to reconnect if not connected
    RCLCPP_DEBUG(getLogger(), "[CpModbusConnection] Not connected, attempting to connect...");
    int rc = modbus_connect(ctx_);
    if (rc == 0)
    {
      connected_ = true;
      RCLCPP_INFO(getLogger(), "[CpModbusConnection] Connection restored");
      onConnectionRestored_();
      if (postConnectionRestoredEvent_)
      {
        postConnectionRestoredEvent_();
      }
    }
    return;
  }

  // Heartbeat: Try to read a single coil to verify connectivity
  uint8_t status;
  int rc = modbus_read_bits(ctx_, 0x0000, 1, &status);

  if (rc == -1)
  {
    connected_ = false;
    std::string error_msg = modbus_strerror(errno);
    RCLCPP_WARN(getLogger(), "[CpModbusConnection] Heartbeat failed: %s", error_msg.c_str());
    onConnectionLost_();
    if (postConnectionLostEvent_)
    {
      postConnectionLostEvent_();
    }
  }
  else
  {
    RCLCPP_DEBUG(getLogger(), "[CpModbusConnection] Heartbeat OK (coil 0 status: %d)", status);
  }
}

}  // namespace cl_modbus_tcp_relay
