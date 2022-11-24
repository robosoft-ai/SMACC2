// Copyright 2021 RobosoftAI Inc.
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

#include <multirole_sensor_client/cl_multirole_sensor.hpp>

#include <string>

#include <smacc2/smacc_client_behavior.hpp>

namespace cl_multirole_sensor
{
template <typename ClientType>
class CbDefaultMultiRoleSensorBehavior : public smacc2::SmaccClientBehavior
{
public:
  typedef typename ClientType::TMessageType TMessageType;

  ClientType * sensor_;

  CbDefaultMultiRoleSensorBehavior() { sensor_ = nullptr; }

  static std::string getEventLabel()
  {
    // show ros message type
    return demangleSymbol(typeid(TMessageType).name());
  }

  std::function<void()> deferedEventPropagation;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    deferedEventPropagation = [this]() {
      RCLCPP_INFO(
        getLogger(), "[CbDefaultMultiRoleSensorBehavior] onEntry. Requires client of type '%s'",
        demangleSymbol<ClientType>().c_str());

      // just propagate the client events from this client behavior source.
      sensor_->onMessageReceived(
        &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent<
          EvTopicMessage<TSourceObject, TOrthogonal>>,
        this);
      sensor_->onFirstMessageReceived(
        &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent<
          EvTopicInitialMessage<TSourceObject, TOrthogonal>>,
        this);
      sensor_->onMessageTimeout(
        &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateEvent2<
          EvTopicMessageTimeout<TSourceObject, TOrthogonal>>,
        this);
    };
  }

  template <typename EvType>
  void propagateEvent(const TMessageType & /*msg*/)
  {
    // TODO: copy event concept fields
    this->postEvent<EvType>();
  }

  template <typename EvType>
  void propagateEvent2()
  {
    this->postEvent<EvType>();
  }

  void onEntry() override
  {
    RCLCPP_INFO(
      getLogger(), "[CbDefaultMultiRoleSensorBehavior] onEntry. Requires client of type '%s'",
      demangleSymbol<ClientType>().c_str());

    if (sensor_ == nullptr)
    {
      this->requiresClient(sensor_);
    }

    if (sensor_ == nullptr)
    {
      RCLCPP_FATAL_STREAM(
        getLogger(),
        "[CbDefaultMultiRoleSensorBehavior]Sensor client behavior needs a client of type: "
          << demangleSymbol<ClientType>() << " but it is not found.");
    }
    else
    {
      deferedEventPropagation();
      RCLCPP_INFO(getLogger(), "[CbDefaultMultiRoleSensorBehavior] onEntry. sensor initialize");
    }
  }

  void onExit() {}

  virtual void onMessageCallback(const TMessageType & /*msg*/)
  {
    // empty to fill by sensor customization based on inheritance
  }
};
}  // namespace cl_multirole_sensor
