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

#include <sm_dance_bot_warehouse/clients/cl_string_publisher/cl_string_publisher.hpp>
#include <smacc2/smacc_client_behavior.hpp>
#include <std_msgs/msg/string.hpp>

namespace sm_dance_bot_warehouse
{
namespace cl_string_publisher
{
class CbStringPublisher : public smacc2::SmaccClientBehavior
{
public:
  ClStringPublisher * publisherClient_;
  std::string msg_;

  CbStringPublisher(std::string msg) { msg_ = msg; }

  virtual void runtimeConfigure() override
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "Creating CbStringPublisher behavior with stored message: " << msg_);
  }

  virtual void onEntry() { this->requiresClient(publisherClient_); }

  void onExit() override
  {
    std_msgs::msg::String rosmsg;
    rosmsg.data = msg_;
    publisherClient_->publish(rosmsg);
  }
};
}  // namespace cl_string_publisher
}  // namespace sm_dance_bot_warehouse
