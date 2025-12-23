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

#include <cl_modbus_tcp_relay/components/cp_modbus_relay.hpp>

namespace cl_modbus_tcp_relay
{

CpModbusRelay::CpModbusRelay() : connectionComponent_(nullptr) {}

CpModbusRelay::~CpModbusRelay() {}

void CpModbusRelay::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpModbusRelay] Initializing...");

  // Require the connection component
  this->requiresComponent(connectionComponent_);

  if (!connectionComponent_)
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Failed to get CpModbusConnection component");
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CpModbusRelay] Initialized with connection component");
  }
}

bool CpModbusRelay::writeCoil(int channel, bool state)
{
  if (!isValidChannel(channel))
  {
    std::string error = "Invalid channel: " + std::to_string(channel) + " (must be 1-8)";
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] %s", error.c_str());
    onWriteFailure_(channel, error);
    if (postWriteFailureEvent_) postWriteFailureEvent_();
    return false;
  }

  if (!connectionComponent_ || !connectionComponent_->isConnected())
  {
    std::string error = "Not connected to Modbus device";
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] %s", error.c_str());
    onWriteFailure_(channel, error);
    if (postWriteFailureEvent_) postWriteFailureEvent_();
    return false;
  }

  std::lock_guard<std::mutex> lock(connectionComponent_->getMutex());
  modbus_t * ctx = connectionComponent_->getContext();

  int address = channelToAddress(channel);
  int value = state ? TRUE : FALSE;

  RCLCPP_INFO(
    getLogger(), "[CpModbusRelay] Writing coil %d (channel %d) = %s", address, channel,
    state ? "ON" : "OFF");

  int rc = modbus_write_bit(ctx, address, value);

  if (rc == -1)
  {
    std::string error = modbus_strerror(errno);
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Write failed: %s", error.c_str());
    onWriteFailure_(channel, error);
    if (postWriteFailureEvent_) postWriteFailureEvent_();
    return false;
  }

  RCLCPP_INFO(getLogger(), "[CpModbusRelay] Write successful");
  onWriteSuccess_(channel, state);
  if (postWriteSuccessEvent_) postWriteSuccessEvent_();
  return true;
}

bool CpModbusRelay::readCoil(int channel, bool & state)
{
  if (!isValidChannel(channel))
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Invalid channel: %d (must be 1-8)", channel);
    return false;
  }

  if (!connectionComponent_ || !connectionComponent_->isConnected())
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Not connected to Modbus device");
    return false;
  }

  std::lock_guard<std::mutex> lock(connectionComponent_->getMutex());
  modbus_t * ctx = connectionComponent_->getContext();

  int address = channelToAddress(channel);
  uint8_t status;

  int rc = modbus_read_bits(ctx, address, 1, &status);

  if (rc == -1)
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Read failed: %s", modbus_strerror(errno));
    return false;
  }

  state = (status != 0);
  RCLCPP_DEBUG(
    getLogger(), "[CpModbusRelay] Read coil %d (channel %d) = %s", address, channel,
    state ? "ON" : "OFF");
  return true;
}

bool CpModbusRelay::writeAllCoils(bool state)
{
  // Create a mask with all bits set or cleared
  uint8_t mask = state ? 0xFF : 0x00;
  return writeAllCoils(mask);
}

bool CpModbusRelay::writeAllCoils(uint8_t mask)
{
  if (!connectionComponent_ || !connectionComponent_->isConnected())
  {
    std::string error = "Not connected to Modbus device";
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] %s", error.c_str());
    onWriteFailure_(0, error);
    if (postWriteFailureEvent_) postWriteFailureEvent_();
    return false;
  }

  std::lock_guard<std::mutex> lock(connectionComponent_->getMutex());
  modbus_t * ctx = connectionComponent_->getContext();

  // Convert bitmask to array of coil states
  uint8_t coils[NUM_CHANNELS];
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    coils[i] = (mask & (1 << i)) ? TRUE : FALSE;
  }

  RCLCPP_INFO(
    getLogger(), "[CpModbusRelay] Writing all %d coils with mask 0x%02X", NUM_CHANNELS, mask);

  int rc = modbus_write_bits(ctx, COIL_BASE_ADDRESS, NUM_CHANNELS, coils);

  if (rc == -1)
  {
    std::string error = modbus_strerror(errno);
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Write all failed: %s", error.c_str());
    onWriteFailure_(0, error);
    if (postWriteFailureEvent_) postWriteFailureEvent_();
    return false;
  }

  RCLCPP_INFO(getLogger(), "[CpModbusRelay] Write all successful");

  // Emit success for each channel that was turned on
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    bool state = (mask & (1 << i)) != 0;
    onWriteSuccess_(i + 1, state);
  }
  if (postWriteSuccessEvent_) postWriteSuccessEvent_();
  return true;
}

bool CpModbusRelay::readAllCoils(uint8_t & states)
{
  if (!connectionComponent_ || !connectionComponent_->isConnected())
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Not connected to Modbus device");
    return false;
  }

  std::lock_guard<std::mutex> lock(connectionComponent_->getMutex());
  modbus_t * ctx = connectionComponent_->getContext();

  uint8_t coils[NUM_CHANNELS];

  int rc = modbus_read_bits(ctx, COIL_BASE_ADDRESS, NUM_CHANNELS, coils);

  if (rc == -1)
  {
    RCLCPP_ERROR(getLogger(), "[CpModbusRelay] Read all failed: %s", modbus_strerror(errno));
    return false;
  }

  // Convert array to bitmask
  states = 0;
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    if (coils[i])
    {
      states |= (1 << i);
    }
  }

  RCLCPP_DEBUG(getLogger(), "[CpModbusRelay] Read all coils: 0x%02X", states);
  onReadComplete_(states);
  return true;
}

}  // namespace cl_modbus_tcp_relay
