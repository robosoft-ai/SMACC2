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

#include <smacc2/client_bases/smacc_publisher_client.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot
{
namespace cl_string_publisher
{
class ClStringPublisher : public smacc2::client_bases::SmaccPublisherClient
{
public:
  ClStringPublisher(std::string topicName) : topicName_(topicName) {}

  void onInitialize() override { this->configure<std_msgs::msg::String>(topicName_); }

  std::string topicName_;
};
}  // namespace cl_string_publisher
}  // namespace sm_dance_bot
