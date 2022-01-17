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

#include <smacc2/client_behaviors/cb_wait_action_server.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbWaitActionServer::CbWaitActionServer(std::chrono::milliseconds timeout) : timeout_(timeout) {}

CbWaitActionServer::~CbWaitActionServer() {}

void CbWaitActionServer::onEntry()
{
  if (client_ != nullptr)
  {
    std::shared_ptr<rclcpp_action::ClientBase> client_base = client_->getClientBase();
    RCLCPP_INFO(getLogger(), "[CbWaitActionServer] waiting action server..");
    bool found = client_base->wait_for_action_server(timeout_);

    if (found)
    {
      RCLCPP_INFO(getLogger(), "[CbWaitActionServer] action server already available");
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbWaitActionServer] action server not found, timeout");
      this->postFailureEvent();
    }
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbWaitActionServer] there is no action client in this orthogonal");
    this->postFailureEvent();
  }
}
}  // namespace client_behaviors
}  // namespace smacc2
