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

#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
using namespace smacc2::client_bases;
using namespace smacc2::client_core_components;

// waits the action server is available in the current orthogonal
template <typename ActionT>
class CbWaitActionServer2 : public smacc2::SmaccAsyncClientBehavior
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresComponent(cp_action_client_, ComponentRequirement::SOFT);
  }

  CbWaitActionServer2(std::chrono::milliseconds timeout) : timeout_(timeout) {}

  virtual ~CbWaitActionServer2() {}

  void onEntry()
  {
    if (cp_action_client_ != nullptr)
    {
      RCLCPP_INFO(
        getLogger(), "[CbWaitActionServer] waiting for action server (using CpActionClient)...");
      bool found = false;
      auto starttime = getNode()->now();
      while (!this->isShutdownRequested() && !found && (getNode()->now() - starttime) < timeout_)
      {
        auto client_base = cp_action_client_->getActionClient();
        found = client_base->wait_for_action_server(std::chrono::milliseconds(1000));
      }

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

private:
  CpActionClient<ActionT> * cp_action_client_;

  std::chrono::milliseconds timeout_;
};
}  // namespace client_behaviors
}  // namespace smacc2
