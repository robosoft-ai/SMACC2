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

#include <multirole_sensor_client/cl_multirole_sensor.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sm_dance_bot_strikes_back/clients/cl_temperature_sensor/cl_temperature_sensor.hpp>
#include <smacc2/smacc_orthogonal.hpp>

using namespace std::chrono_literals;

namespace sm_dance_bot_strikes_back
{
class OrTemperatureSensor : public smacc2::Orthogonal<OrTemperatureSensor>
{
public:
  void onInitialize() override
  {
    auto clTemperatureSensor = this->createClient<ClTemperatureSensor>();

    clTemperatureSensor->topicName = "/temperature";
    clTemperatureSensor->timeout_ = rclcpp::Duration(10s);
  }
};
}  // namespace sm_dance_bot_strikes_back
