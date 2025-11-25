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
#include <sm_three_some/clients/cl_subscriber/cl_subscriber.hpp>
#include <cl_generic_sensor/client_behaviors/cb_default_generic_sensor_behavior.hpp>

namespace sm_three_some
{
namespace cl_subscriber
{
// Watchdog subscriber behavior with timeout monitoring
// Uses the multirole sensor behavior for component-based architecture
class CbWatchdogSubscriberBehavior
  : public cl_generic_sensor::CbDefaultGenericSensorBehavior<ClSubscriber>
{
public:
  typedef std_msgs::msg::UInt16 TMessageType;

  void onEntry() override
  {
    // Call base class onEntry to setup component connections
    cl_generic_sensor::CbDefaultGenericSensorBehavior<ClSubscriber>::onEntry();

    RCLCPP_INFO(getLogger(), "[CbWatchdogSubscriberBehavior] Watchdog behavior active");

    // The timeout monitoring is handled automatically by the CpMessageTimeout component
    // if it was configured in the ClSubscriber client
  }
};
}  // namespace cl_subscriber
}  // namespace sm_three_some
