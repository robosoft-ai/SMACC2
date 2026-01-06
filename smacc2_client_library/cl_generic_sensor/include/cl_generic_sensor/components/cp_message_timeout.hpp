// Copyright 2025 Robosoft Inc.
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

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

namespace cl_generic_sensor
{
namespace components
{
using namespace smacc2;

// Timeout event definition
template <typename TSource, typename TOrthogonal>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<TSource, TOrthogonal>>
{
};

// Component that monitors topic message reception and posts timeout events
// when messages are not received within the specified duration
template <typename MessageType>
class CpMessageTimeout : public smacc2::ISmaccComponent
{
public:
  typedef MessageType TMessageType;

  // Timeout signal - triggered when timeout occurs
  SmaccSignal<void()> onMessageTimeout_;

  // Optional timeout duration configuration
  std::optional<rclcpp::Duration> timeout_;

  CpMessageTimeout() : initialized_(false) {}

  virtual ~CpMessageTimeout() {}

  // Signal subscription method for timeout events
  template <typename T>
  smacc2::SmaccSignalConnection onMessageTimeout(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onMessageTimeout_, callback, object);
  }

  // Function to post timeout event - set during orthogonal allocation
  std::function<void()> postTimeoutMessageEvent;

  // Template method called during state orthogonal allocation
  template <typename TOrthogonal, typename TSourceObject>
  void onComponentInitialization()
  {
    // Setup the event posting function with proper template types
    this->postTimeoutMessageEvent = [this]()
    {
      // Emit signal
      this->onMessageTimeout_();

      // Post SMACC event
      auto event = new EvTopicMessageTimeout<TSourceObject, TOrthogonal>();
      this->postEvent(event);
    };

    // Require the CpTopicSubscriber component to monitor messages
    smacc2::client_core_components::CpTopicSubscriber<MessageType> * subscriberComponent;
    this->requiresComponent(subscriberComponent);

    // Subscribe to message received signal to reset timer
    subscriberComponent->onMessageReceived(&CpMessageTimeout<MessageType>::resetTimer, this);
  }

  // Initialize the timeout timer
  void onInitialize() override
  {
    if (!initialized_)
    {
      if (timeout_)
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName()
                           << "] Initializing message timeout watchdog with duration: "
                           << timeout_->seconds() << "s");

        // Create the timeout timer
        timeoutTimer_ = rclcpp::create_timer(
          this->getNode(), this->getNode()->get_clock(), *timeout_,
          std::bind(&CpMessageTimeout<MessageType>::timeoutCallback, this));

        timeoutTimer_->reset();
      }
      else
      {
        RCLCPP_WARN(
          getLogger(),
          "[CpMessageTimeout] Timeout duration not set, skipping timeout watchdog functionality");
      }

      initialized_ = true;
    }
  }

protected:
  // Reset the timer when a message is received
  void resetTimer(const MessageType & /*msg*/)
  {
    if (timeoutTimer_)
    {
      timeoutTimer_->reset();
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timeoutTimer_;
  bool initialized_;

  // Timer callback - invoked when timeout occurs
  void timeoutCallback()
  {
    RCLCPP_WARN(
      getLogger(), "[CpMessageTimeout] Message timeout occurred after %f seconds",
      timeout_->seconds());

    if (postTimeoutMessageEvent)
    {
      postTimeoutMessageEvent();
    }
  }
};

}  // namespace components
}  // namespace cl_generic_sensor
