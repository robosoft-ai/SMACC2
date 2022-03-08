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

#include <smacc2/client_base_components/cp_topic_publisher.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int8.hpp>

namespace sm_husky_barrel_search_1
{
namespace cl_led_array
{
class ClLedArray : public smacc2::ISmaccClient
{
public:
  smacc2::components::CpTopicPublisher<std_msgs::msg::Int8> * greenLed_;
  smacc2::components::CpTopicPublisher<std_msgs::msg::Int8> * yellowLed_;
  smacc2::components::CpTopicPublisher<std_msgs::msg::Int8> * redLed_;

  ClLedArray() {}

  virtual ~ClLedArray() {}

  void onInitialize() override
  {
    greenLed_ =
      getComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>("greenLed");
    yellowLed_ =
      getComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>("yellowLed");
    redLed_ = getComponent<smacc2::components::CpTopicPublisher<std_msgs::msg::Int8>>("redLed");
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
  }
};
}  // namespace cl_led_array
}  // namespace sm_husky_barrel_search_1
