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
#include <cl_generic_sensor/cl_generic_sensor.hpp>

#include <std_msgs/msg/int16.hpp>

namespace sm_three_some
{
namespace cl_subscriber
{
// Component-based subscriber client using ClGenericSensor
// This demonstrates the pure component-based architecture
class ClSubscriber : public cl_generic_sensor::ClGenericSensor<std_msgs::msg::UInt16>
{
public:
  ClSubscriber()
  {
    // Configure the topic to subscribe to
    // Note: This subscribes to the keyboard unicode topic for testing
    this->topicName_ = "/keyboard_unicode";

    // Optional: Configure timeout for watchdog functionality
    // Uncomment the line below to enable 5-second timeout
    // this->timeout_ = rclcpp::Duration(5, 0);
  }
};
}  // namespace cl_subscriber
}  // namespace sm_three_some
