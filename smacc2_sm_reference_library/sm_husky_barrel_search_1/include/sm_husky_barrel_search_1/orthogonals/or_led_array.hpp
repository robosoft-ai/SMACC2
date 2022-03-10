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

#include <smacc2/smacc_orthogonal.hpp>
#include <sm_husky_barrel_search_1/clients/led_array/cl_led_array.hpp>

namespace sm_husky_barrel_search_1
{
using namespace std::chrono_literals;

class OrLedArray : public smacc2::Orthogonal<OrLedArray>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_led_array::ClLedArray>();

    client->createNamedComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>(
      "greenLed", "/light_model_green/cmdled");

    client->createNamedComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>(
      "yellowLed", "/light_model_yellow/cmdled");

    client->createNamedComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>(
      "redLed", "/light_model_red/cmdled");
  }
};
}  // namespace sm_husky_barrel_search_1
