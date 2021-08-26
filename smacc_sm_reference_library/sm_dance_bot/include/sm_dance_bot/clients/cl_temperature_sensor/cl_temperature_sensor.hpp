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

#include <multirole_sensor_client/cl_multirole_sensor.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot
{
namespace cl_temperature_sensor
{
class ClTemperatureSensor
: public cl_multirole_sensor::ClMultiroleSensor<sensor_msgs::msg::Temperature>
{
public:
  ClTemperatureSensor() {}
};
}  // namespace cl_temperature_sensor
}  // namespace sm_dance_bot
