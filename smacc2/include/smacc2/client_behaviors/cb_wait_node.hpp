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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
using namespace std::chrono_literals;

// Asynchronous behavior that waits to a topic message to send EvCbSuccess event
// a guard function can be set to use conditions on the contents
class CbWaitNode : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbWaitNode(std::string nodeName);

  void onEntry() override;

protected:
  std::string nodeName_;

  rclcpp::Rate rate_;
};
}  // namespace client_behaviors
}  // namespace smacc2
