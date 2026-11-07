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
class CbDeactivateOnExit : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbDeactivateOnExit() {}
  virtual ~CbDeactivateOnExit() {}

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
    eventMonitor_->onTransitionOnDeactivateSuccess(&CbDeactivateOnExit::onTransitionSuccess, this);
    eventMonitor_->onTransitionOnDeactivateFailure(&CbDeactivateOnExit::onTransitionFailure, this);
    eventMonitor_->onTransitionOnDeactivateError(&CbDeactivateOnExit::onTransitionFailure, this);
  }

  virtual void onExit() override
  {
    // Runs on this behavior's onExit thread; the wait ends on the response, on a finish request
    // or on ROS shutdown. A rejection is logged by the client; the state is already leaving, so
    // nothing is posted here.
    lifecycleNodeClient_->deactivate([this]() { return this->isShutdownRequested(); });
  }

  void onTransitionSuccess()
  {
    RCLCPP_INFO(getLogger(), "[CbDeactivateOnExit] Deactivate transition succeeded");
    this->postSuccessEvent();
  }

  void onTransitionFailure()
  {
    RCLCPP_WARN(getLogger(), "[CbDeactivateOnExit] Deactivate transition failed or errored");
    this->postFailureEvent();
  }

private:
  ClLifecycleNode * lifecycleNodeClient_;
  CpLifecycleEventMonitor * eventMonitor_;
};
}  // namespace cl_lifecycle_node
