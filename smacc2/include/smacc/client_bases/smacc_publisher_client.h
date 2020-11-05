/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <optional>

namespace smacc
{
namespace client_bases
{
class SmaccPublisherClient : public smacc::ISmaccClient
{
public:
  std::optional<std::string> topicName;
  std::optional<int> queueSize;

  SmaccPublisherClient();
  virtual ~SmaccPublisherClient();

  template <typename MessageType>
  void configure(std::string topicName)
  {
    this->topicName = topicName;
    if (!initialized_)
    {
      if (!this->topicName)
      {
        RCLCPP_ERROR(getNode()->get_logger(), "topic publisher with no topic name set. Skipping advertising.");
        return;
      }
      
      if (!queueSize)
        queueSize = 1;
      rclcpp::SensorDataQoS qos;
      qos.keep_last(*queueSize);

      RCLCPP_INFO_STREAM(getNode()->get_logger(), "[" << this->getName() << "] Client Publisher to topic: " << topicName);
      pub_ = getNode()->create_publisher<MessageType>(*(this->topicName), qos);

      this->initialized_ = true;
    }
  }

  template <typename MessageType>
  void publish(const MessageType &msg)
  {
    //pub_->publish(msg);
    std::dynamic_pointer_cast<rclcpp::Publisher<MessageType>>(pub_)->publish(msg);
  }

  rclcpp::PublisherBase::SharedPtr pub_;

private:
  bool initialized_;
};
} // namespace client_bases
} // namespace smacc