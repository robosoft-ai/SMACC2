/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <smacc/smacc_signal.h>
#include <optional>

namespace smacc
{
namespace client_bases
{
using namespace smacc::default_events;

template <typename MessageType>
class SmaccSubscriberClient : public smacc::ISmaccClient
{
public:
  std::optional<std::string> topicName;
  std::optional<int> queueSize;

  typedef MessageType TMessageType;

  SmaccSubscriberClient() { initialized_ = false; }

  SmaccSubscriberClient(std::string topicname) { topicName = topicname; }

  virtual ~SmaccSubscriberClient()
  {
    // sub_.reset() // not needed
  }

  smacc::SmaccSignal<void(const MessageType &)> onFirstMessageReceived_;
  smacc::SmaccSignal<void(const MessageType &)> onMessageReceived_;

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
      auto event = new EvTopicMessage<TSourceObject, TOrthogonal>();
      event->msgData = msg;
      this->postEvent(event);
    };

    this->postInitialMessageEvent = [=](auto msg) {
      auto event = new EvTopicInitialMessage<TSourceObject, TOrthogonal>();
      event->msgData = msg;
      this->postEvent(event);
    };
  }

protected:
  virtual void onInitialize() override
  {
    if (!initialized_)
    {
      firstMessage_ = true;

      if (!queueSize) queueSize = 1;

      if (!topicName)
      {
        RCLCPP_ERROR(
          getNode()->get_logger(), "topic client with no topic name set. Skipping subscribing");
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getNode()->get_logger(),
          "[" << this->getName() << "] Subscribing to topic: " << *topicName);

        rclcpp::SensorDataQoS qos;
        if (queueSize) qos.keep_last(*queueSize);

        std::function<void(typename MessageType::SharedPtr)> fn = [=](auto msg) {
          this->messageCallback(*msg);
        };
        sub_ = getNode()->create_subscription<MessageType>(*topicName, qos, fn);
        this->initialized_ = true;
      }
    }
  }

private:
  typename rclcpp::Subscription<MessageType>::SharedPtr sub_;
  bool firstMessage_;
  bool initialized_;

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
}  // namespace client_bases
}  // namespace smacc
