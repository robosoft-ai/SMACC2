// Copyright 2025 Robosoft Inc.
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

#include <cl_lifecycle_node/cl_lifecycle_node.hpp>
#include <cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_lifecycle_node
{
class CbConfigure : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbConfigure() {}
  virtual ~CbConfigure() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresClient(lifecycleNodeClient_);
    this->requiresComponent(eventMonitor_);
    // State-scoped signal connections, made on the state machine thread. Each is tracked by this
    // behavior's address and finalized when the behavior is disposed at state exit, so a
    // transition event that arrives after the exit cannot call into a destroyed behavior. The
    // raw connect() calls this replaces were never disconnected (finding T).
    eventMonitor_->onTransitionOnConfigureSuccess(&CbConfigure::onTransitionSuccess, this);
    eventMonitor_->onTransitionOnConfigureFailure(&CbConfigure::onTransitionFailure, this);
    eventMonitor_->onTransitionOnConfigureError(&CbConfigure::onTransitionFailure, this);
  }

  virtual void onEntry() override
  {
    // Runs on this behavior's thread. The wait inside the client ends when the request is
    // answered, when this behavior is asked to finish (state exit, shutdown) or when ROS shuts
    // down. If the request was abandoned nothing is posted: the state is already leaving. If the
    // node rejected it, no transition event will ever arrive for this attempt, so the behavior
    // reports the failure itself.
    const bool accepted =
      lifecycleNodeClient_->configure([this]() { return this->isShutdownRequested(); });
    if (!accepted && !this->isShutdownRequested())
    {
      this->postFailureEvent();
    }
  }

  void onTransitionSuccess()
  {
    RCLCPP_INFO(getLogger(), "[CbConfigure] Configure transition succeeded");
    this->postSuccessEvent();
  }

  void onTransitionFailure()
  {
    RCLCPP_WARN(getLogger(), "[CbConfigure] Configure transition failed or errored");
    this->postFailureEvent();
  }

private:
  ClLifecycleNode * lifecycleNodeClient_;
  CpLifecycleEventMonitor * eventMonitor_;
};
}  // namespace cl_lifecycle_node
