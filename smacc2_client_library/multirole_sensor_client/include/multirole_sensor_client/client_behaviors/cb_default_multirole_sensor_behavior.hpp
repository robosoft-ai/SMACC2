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
#include <multirole_sensor_client/components/cp_message_timeout.hpp>

#include <string>

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace cl_multirole_sensor
{
// Component-based default behavior for multirole sensor client
// This behavior works with ClMultiroleSensor and propagates events from components
template <typename ClientType>
class CbDefaultMultiRoleSensorBehavior : public smacc2::SmaccClientBehavior
{
public:
  typedef typename ClientType::TMessageType TMessageType;

  ClientType * sensor_;

  // References to the components we'll use
  smacc2::client_core_components::CpTopicSubscriber<TMessageType> * subscriberComponent_;
  cl_multirole_sensor::components::CpMessageTimeout<TMessageType> * timeoutComponent_;

  CbDefaultMultiRoleSensorBehavior()
  : sensor_(nullptr), subscriberComponent_(nullptr), timeoutComponent_(nullptr)
  {
  }

  static std::string getEventLabel()
  {
    // Show ros message type
    return smacc2::demangleSymbol(typeid(TMessageType).name());
  }

  // Deferred event propagation functions - set during orthogonal allocation
  std::function<void()> deferredComponentConnection;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // Setup deferred component connection and event propagation
    deferredComponentConnection = [this]()
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbDefaultMultiRoleSensorBehavior] Connecting to components for client type '%s'",
        smacc2::demangleSymbol<ClientType>().c_str());

      // Get the subscriber component from the client
      this->requiresComponent(subscriberComponent_);

      if (subscriberComponent_ == nullptr)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[CbDefaultMultiRoleSensorBehavior] Failed to get CpTopicSubscriber component!");
        return;
      }

      // Connect to subscriber component signals to propagate events
      subscriberComponent_->onMessageReceived(
        &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateMessageEvent<
          smacc2::default_events::EvTopicMessage<TSourceObject, TOrthogonal, TMessageType>>,
        this);

      subscriberComponent_->onFirstMessageReceived(
        &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateMessageEvent<
          smacc2::default_events::EvTopicInitialMessage<TSourceObject, TOrthogonal, TMessageType>>,
        this);

      RCLCPP_INFO(
        getLogger(), "[CbDefaultMultiRoleSensorBehavior] Connected to subscriber component");

      // Try to get the timeout component (it's optional)
      try
      {
        this->requiresComponent(timeoutComponent_);

        if (timeoutComponent_ != nullptr)
        {
          // Connect to timeout component signal to propagate timeout events
          timeoutComponent_->onMessageTimeout(
            &CbDefaultMultiRoleSensorBehavior<ClientType>::propagateTimeoutEvent<
              cl_multirole_sensor::components::EvTopicMessageTimeout<TSourceObject, TOrthogonal>>,
            this);

          RCLCPP_INFO(
            getLogger(),
            "[CbDefaultMultiRoleSensorBehavior] Connected to timeout component (watchdog active)");
        }
      }
      catch (const std::exception & e)
      {
        RCLCPP_INFO(
          getLogger(),
          "[CbDefaultMultiRoleSensorBehavior] Timeout component not found (watchdog disabled): %s",
          e.what());
      }
    };
  }

  // Propagate message event with message data
  template <typename EvType>
  void propagateMessageEvent(const TMessageType & msg)
  {
    RCLCPP_DEBUG(
      getLogger(), "[CbDefaultMultiRoleSensorBehavior] Propagating message event: %s",
      smacc2::demangleSymbol<EvType>().c_str());

    auto event = new EvType();
    event->msgData = msg;
    this->postEvent(event);
  }

  // Propagate timeout event (no message data)
  template <typename EvType>
  void propagateTimeoutEvent()
  {
    RCLCPP_WARN(
      getLogger(), "[CbDefaultMultiRoleSensorBehavior] Propagating timeout event: %s",
      smacc2::demangleSymbol<EvType>().c_str());

    this->postEvent<EvType>();
  }

  void onEntry() override
  {
    RCLCPP_INFO(
      getLogger(), "[CbDefaultMultiRoleSensorBehavior] onEntry. Requires client of type '%s'",
      smacc2::demangleSymbol<ClientType>().c_str());

    // Get the client if we don't have it yet
    if (sensor_ == nullptr)
    {
      this->requiresClient(sensor_);
    }

    if (sensor_ == nullptr)
    {
      RCLCPP_FATAL_STREAM(
        getLogger(),
        "[CbDefaultMultiRoleSensorBehavior] Sensor client behavior needs a client of type: "
          << smacc2::demangleSymbol<ClientType>() << " but it is not found.");
    }
    else
    {
      // Connect to components
      deferredComponentConnection();
      RCLCPP_INFO(getLogger(), "[CbDefaultMultiRoleSensorBehavior] Sensor behavior initialized");
    }
  }

  void onExit() override { RCLCPP_INFO(getLogger(), "[CbDefaultMultiRoleSensorBehavior] onExit"); }

  // Virtual callback for custom message processing in derived behaviors
  virtual void onMessageCallback(const TMessageType & /*msg*/)
  {
    // Empty to fill by sensor customization based on inheritance
  }
};

}  // namespace cl_multirole_sensor
