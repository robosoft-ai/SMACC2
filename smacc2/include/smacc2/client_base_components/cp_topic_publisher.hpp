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
#include <rclcpp/rclcpp.hpp>
#include <smacc2/common.hpp>
#include <smacc2/component.hpp>

namespace smacc2
{
namespace components
{
using namespace smacc2::default_events;

template <typename MessageType>
class CpTopicPublisher : public smacc2::ISmaccComponent
{
public:
  std::optional<int> queueSize;
  std::optional<rmw_qos_durability_policy_t> durability;
  std::optional<rmw_qos_reliability_policy_t> reliability;

  typedef MessageType TMessageType;

  CpTopicPublisher(std::string topicname)
  {
    this->topicName_ = topicname;
    initialized_ = false;
  }

  virtual ~CpTopicPublisher() {}

  void publish(const MessageType & msg) { pub_->publish(msg); }

  void onInitialize() override;

private:
  typename rclcpp::Publisher<MessageType>::SharedPtr pub_;
  //rclcpp::PublisherBase::SharedPtr pub_;

  bool initialized_;
  std::string topicName_;
};

template <typename T>
void CpTopicPublisher<T>::onInitialize()
{
  if (!initialized_)
  {
    if (!queueSize) queueSize = 1;
    if (!durability) durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    if (!reliability) reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    rclcpp::SensorDataQoS qos;
    qos.keep_last(*queueSize);
    qos.durability(*durability);
    qos.reliability(*reliability);

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Publisher to topic: " << topicName_);

    auto nh = this->getNode();
    pub_ = nh->template create_publisher<T>(this->topicName_, qos);

    this->initialized_ = true;
  }
}

}  // namespace components
}  // namespace smacc2
