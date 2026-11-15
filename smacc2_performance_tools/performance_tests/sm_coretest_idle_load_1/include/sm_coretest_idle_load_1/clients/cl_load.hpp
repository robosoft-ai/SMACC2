// Copyright 2026 RobosoftAI Inc.
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

#include <smacc2/client_core_components/cp_ros2_timer.hpp>
#include <smacc2/client_core_components/cp_topic_publisher.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/string.hpp>

#include <sm_coretest_idle_load_1/components/cp_load_driver.hpp>

namespace sm_coretest_idle_load_1
{
// A 500 Hz timer, a publisher and a subscriber on one topic, and the component that joins them.
class ClLoad : public smacc2::ISmaccClient
{
public:
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    using namespace smacc2::client_core_components;
    const std::string topic = "/sm_coretest_idle_load_1/data";
    this->createComponent<CpRos2Timer, TOrthogonal, TClient>(
      rclcpp::Duration(std::chrono::milliseconds(2)));
    this->createComponent<CpTopicPublisher<std_msgs::msg::String>, TOrthogonal, TClient>(topic);
    this->createComponent<CpTopicSubscriber<std_msgs::msg::String>, TOrthogonal, TClient>(topic);
    this->createComponent<CpLoadDriver, TOrthogonal, TClient>();
  }
};
}  // namespace sm_coretest_idle_load_1
