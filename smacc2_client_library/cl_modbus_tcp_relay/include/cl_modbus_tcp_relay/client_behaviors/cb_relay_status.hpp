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

#include <optional>

namespace cl_modbus_tcp_relay
{

/**
 * @brief Client behavior to read relay channel status
 *
 * Can read single channel or all channels.
 * Posts EvCbSuccess on successful read, EvCbFailure on error.
 */
class CbRelayStatus : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Construct behavior to read all channel statuses
   */
  CbRelayStatus() : channel_(std::nullopt) {}

  /**
   * @brief Construct behavior to read a specific channel status
   * @param channel Relay channel number (1-8)
   */
  explicit CbRelayStatus(int channel) : channel_(channel) {}

  virtual ~CbRelayStatus() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresClient(client_);
    this->requiresComponent(relayComponent_);
  }

  virtual void onEntry() override
  {
    if (channel_.has_value())
    {
      readSingleChannel(channel_.value());
    }
    else
    {
      readAllChannels();
    }
  }

  /**
   * @brief Get the last read channel states (bitmask)
   * @return Bitmask of channel states (bit 0 = channel 1, etc.)
   */
  uint8_t getChannelStates() const { return channelStates_; }

  /**
   * @brief Virtual callback for status read completion
   * Override in derived classes for custom handling
   * @param channelStates Bitmask of channel states
   */
  virtual void onStatusRead(uint8_t channelStates)
  {
    RCLCPP_INFO(getLogger(), "[CbRelayStatus] Status read callback: 0x%02X", channelStates);
  }

private:
  std::optional<int> channel_;
  ClModbusTcpRelay * client_;
  CpModbusRelay * relayComponent_;
  uint8_t channelStates_ = 0;

  void readSingleChannel(int channel)
  {
    RCLCPP_INFO(getLogger(), "[CbRelayStatus] Reading channel %d status", channel);

    if (!relayComponent_)
    {
      RCLCPP_ERROR(getLogger(), "[CbRelayStatus] Relay component not available");
      this->postFailureEvent();
      return;
    }

    bool state;
    bool success = relayComponent_->readCoil(channel, state);

    if (success)
    {
      channelStates_ = state ? (1 << (channel - 1)) : 0;
      RCLCPP_INFO(getLogger(), "[CbRelayStatus] Channel %d is %s", channel, state ? "ON" : "OFF");
      onStatusRead(channelStates_);
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbRelayStatus] Failed to read channel %d", channel);
      this->postFailureEvent();
    }
  }

  void readAllChannels()
  {
    RCLCPP_INFO(getLogger(), "[CbRelayStatus] Reading all channel statuses");

    if (!relayComponent_)
    {
      RCLCPP_ERROR(getLogger(), "[CbRelayStatus] Relay component not available");
      this->postFailureEvent();
      return;
    }

    bool success = relayComponent_->readAllCoils(channelStates_);

    if (success)
    {
      RCLCPP_INFO(getLogger(), "[CbRelayStatus] All channel statuses: 0x%02X", channelStates_);

      // Log individual channel states
      for (int i = 0; i < 8; i++)
      {
        bool state = (channelStates_ & (1 << i)) != 0;
        RCLCPP_INFO(getLogger(), "[CbRelayStatus]   Channel %d: %s", i + 1, state ? "ON" : "OFF");
      }

      onStatusRead(channelStates_);
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbRelayStatus] Failed to read channel statuses");
      this->postFailureEvent();
    }
  }
};

}  // namespace cl_modbus_tcp_relay
