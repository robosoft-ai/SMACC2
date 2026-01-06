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
class CbCleanup : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbCleanup() {}
  virtual ~CbCleanup() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresClient(this->lifecycleNodeClient_);
    this->requiresComponent(eventMonitor_);

    eventMonitor_->onTransitionOnCleanupSuccess_.connect([this]() { this->postSuccessEvent(); });
    eventMonitor_->onTransitionOnCleanupFailure_.connect([this]() { this->postFailureEvent(); });
    eventMonitor_->onTransitionOnCleanupError_.connect([this]() { this->postFailureEvent(); });
  }

  virtual void onEntry() override { lifecycleNodeClient_->cleanup(); }

private:
  ClLifecycleNode * lifecycleNodeClient_;
  CpLifecycleEventMonitor * eventMonitor_;
};
}  // namespace cl_lifecycle_node
