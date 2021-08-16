#pragma once
#include <smacc/client_bases/smacc_subscriber_client.h>
#include <smacc/component.h>
#include <smacc/smacc_signal.h>
#include <optional>

namespace smacc
{
namespace components
{
using namespace smacc::default_events;

template <typename MessageType>
class CpTopicSubscriber : public smacc::ISmaccComponent
{
public:
  std::optional<std::string> topicName;
  std::optional<int> queueSize;

  typedef MessageType TMessageType;

  CpTopicSubscriber() { initialized_ = false; }

  CpTopicSubscriber(std::string topicname) { topicName = topicname; }

  virtual ~CpTopicSubscriber() { sub_.shutdown(); }

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

  virtual void initialize()
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
          "[" << this->getName() << "] Subscribing to topic: " << topicName);

        sub_ = getNode()->subscribe(
          *topicName, *queueSize, &CpTopicSubscriber<MessageType>::messageCallback, this);
        this->initialized_ = true;
      }
    }
  }

protected:
  rclcpp::Node::SharedPtr getNode();

private:
  ros::Subscriber sub_;
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
}  // namespace smacc