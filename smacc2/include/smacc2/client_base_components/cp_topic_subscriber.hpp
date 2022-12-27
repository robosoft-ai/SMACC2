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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <optional>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

namespace smacc2
{
namespace components
{
using namespace smacc2::default_events;

template <typename MessageType>
class CpTopicSubscriber : public smacc2::ISmaccComponent
{
public:
  std::optional<int> queueSize;

  typedef MessageType TMessageType;

  CpTopicSubscriber() { initialized_ = false; }

  CpTopicSubscriber(std::string topicname) { topicName_ = topicname; }

  virtual ~CpTopicSubscriber() {}

  smacc2::SmaccSignal<void(const MessageType &)> onFirstMessageReceived_;
  smacc2::SmaccSignal<void(const MessageType &)> onMessageReceived_;

  std::function<void(const MessageType &)> postMessageEvent;
  std::function<void(const MessageType &)> postInitialMessageEvent;

  template <typename T>
  boost::signals2::connection onMessageReceived(
    void (T::*callback)(const MessageType &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onMessageReceived_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onFirstMessageReceived(
    void (T::*callback)(const MessageType &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(
      onFirstMessageReceived_, callback, object);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postMessageEvent = [=](auto msg) {
      auto event = new EvTopicMessage<TSourceObject, TOrthogonal, MessageType>();
      event->msgData = msg;
      this->postEvent(event);
    };

    this->postInitialMessageEvent = [=](auto msg) {
      auto event = new EvTopicInitialMessage<TSourceObject, TOrthogonal, MessageType>();
      event->msgData = msg;
      this->postEvent(event);
    };
  }

  void onInitialize() override
  {
    if (!initialized_)
    {
      firstMessage_ = true;

      if (!queueSize) queueSize = 1;

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << this->getName() << "] Subscribing to topic: " << topicName_);

      rclcpp::SensorDataQoS qos;
      if (queueSize) qos.keep_last(*queueSize);

      std::function<void(typename MessageType::SharedPtr)> fn = [this](auto msg) {
        this->messageCallback(*msg);
      };

      sub_ = this->getNode()->template create_subscription<MessageType>(topicName_, qos, fn);
      this->initialized_ = true;
    }
  }

private:
  typename rclcpp::Subscription<MessageType>::SharedPtr sub_;
  bool firstMessage_;
  bool initialized_;
  std::string topicName_;

  void messageCallback(const MessageType & msg)
  {
    if (firstMessage_)
    {
      postInitialMessageEvent(msg);
      onFirstMessageReceived_(msg);
      firstMessage_ = false;
    }

    postMessageEvent(msg);
    onMessageReceived_(msg);
  }
};
}  // namespace components
}  // namespace smacc2
