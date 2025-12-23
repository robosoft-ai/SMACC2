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
#include <smacc2/smacc.hpp>

#include <cstdint>
#include <functional>

namespace cl_modbus_tcp_relay
{

// Forward declarations for events
template <typename TSource, typename TOrthogonal>
struct EvRelayWriteSuccess;

template <typename TSource, typename TOrthogonal>
struct EvRelayWriteFailure;

/**
 * @brief Component that handles Modbus coil read/write operations for 8-channel relay
 *
 * Responsibilities:
 * - Write single coil (channel on/off)
 * - Write all coils simultaneously
 * - Read single coil status
 * - Read all coil statuses
 * - Emit operation result signals
 *
 * Channel numbering: 1-8 (user-friendly)
 * Coil addresses: 0x0000-0x0007 (0-indexed internally)
 */
class CpModbusRelay : public smacc2::ISmaccComponent
{
public:
  static constexpr int NUM_CHANNELS = 8;
  static constexpr int COIL_BASE_ADDRESS = 0x0000;

  CpModbusRelay();
  virtual ~CpModbusRelay();

  void onInitialize() override;

  /**
   * @brief Configure component for event posting during orthogonal allocation
   */
  template <typename TOrthogonal, typename TClient>
  void onStateOrthogonalAllocation()
  {
    postWriteSuccessEvent_ = [this]()
    { this->template postEvent<EvRelayWriteSuccess<CpModbusRelay, TOrthogonal>>(); };

    postWriteFailureEvent_ = [this]()
    { this->template postEvent<EvRelayWriteFailure<CpModbusRelay, TOrthogonal>>(); };
  }

  // Single channel operations (channel 1-8)
  bool writeCoil(int channel, bool state);
  bool readCoil(int channel, bool & state);

  // All channels operations
  bool writeAllCoils(bool state);
  bool writeAllCoils(uint8_t mask);
  bool readAllCoils(uint8_t & states);

  // Signals for operation results
  smacc2::SmaccSignal<void(int channel, bool state)> onWriteSuccess_;
  smacc2::SmaccSignal<void(int channel, const std::string & error)> onWriteFailure_;
  smacc2::SmaccSignal<void(uint8_t states)> onReadComplete_;

  // Signal connection helpers
  template <typename T>
  smacc2::SmaccSignalConnection onWriteSuccess(void (T::*callback)(int, bool), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onWriteSuccess_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onWriteFailure(
    void (T::*callback)(int, const std::string &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onWriteFailure_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onReadComplete(void (T::*callback)(uint8_t), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onReadComplete_, callback, object);
  }

private:
  CpModbusConnection * connectionComponent_;

  std::function<void()> postWriteSuccessEvent_;
  std::function<void()> postWriteFailureEvent_;

  // Helper to convert channel (1-8) to coil address (0-7)
  int channelToAddress(int channel) const { return COIL_BASE_ADDRESS + (channel - 1); }

  // Validate channel number
  bool isValidChannel(int channel) const { return channel >= 1 && channel <= NUM_CHANNELS; }
};

}  // namespace cl_modbus_tcp_relay
