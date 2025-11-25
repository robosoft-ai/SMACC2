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

#include <cl_generic_sensor/components/cp_message_timeout.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc_client.hpp>

namespace cl_generic_sensor
{
using namespace smacc2;

// Pure component-based multirole sensor client
// This client follows the ClKeyboard pattern where all functionality
// is implemented through composable components
template <typename MessageType>
class ClGenericSensor : public smacc2::ISmaccClient
{
public:
  typedef MessageType TMessageType;

  // Optional topic name - if not set, must be configured during component initialization
  std::optional<std::string> topicName_;

  // Optional timeout duration - if not set, timeout functionality is disabled
  std::optional<rclcpp::Duration> timeout_;

  ClGenericSensor() {}

  // Constructor with topic name
  ClGenericSensor(std::string topicName) : topicName_(topicName) {}

  // Constructor with topic name and timeout
  ClGenericSensor(std::string topicName, rclcpp::Duration timeout)
  : topicName_(topicName), timeout_(timeout)
  {
  }

  virtual ~ClGenericSensor() {}

  // Component-based architecture initialization
  // This method creates and configures the components that provide
  // the client's functionality
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    RCLCPP_INFO(getLogger(), "[ClGenericSensor] Initializing component-based sensor client");

    // Create the topic subscriber component
    // This component handles ROS topic subscription and posts SMACC events
    if (!topicName_)
    {
      RCLCPP_ERROR(
        getLogger(),
        "[ClGenericSensor] Topic name not set. Please configure topicName_ before "
        "initialization.");
      return;
    }

    this->createComponent<
      smacc2::client_core_components::CpTopicSubscriber<MessageType>, TOrthogonal, ClGenericSensor>(
      *topicName_);

    RCLCPP_INFO(
      getLogger(), "[ClGenericSensor] Created CpTopicSubscriber for topic: %s",
      topicName_->c_str());

    // Create the message timeout component (optional - only if timeout is configured)
    // This component monitors message reception and posts timeout events
    if (timeout_)
    {
      auto timeoutComponent = this->createComponent<
        cl_generic_sensor::components::CpMessageTimeout<MessageType>, TOrthogonal,
        ClGenericSensor>();

      // Configure the timeout duration
      timeoutComponent->timeout_ = timeout_;

      RCLCPP_INFO(
        getLogger(), "[ClGenericSensor] Created CpMessageTimeout with duration: %f seconds",
        timeout_->seconds());
    }
    else
    {
      RCLCPP_INFO(
        getLogger(), "[ClGenericSensor] Timeout not configured - watchdog functionality disabled");
    }
  }

  // Convenience method to set topic name after construction
  void setTopicName(const std::string & topicName) { topicName_ = topicName; }

  // Convenience method to set timeout after construction
  void setTimeout(const rclcpp::Duration & timeout) { timeout_ = timeout; }
};

}  // namespace cl_generic_sensor
