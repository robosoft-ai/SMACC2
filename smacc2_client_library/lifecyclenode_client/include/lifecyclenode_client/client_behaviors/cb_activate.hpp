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

#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <lifecyclenode_client/lifecyclenode_client.hpp>

namespace cl_lifecyclenode
{
class CbActivate : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbActivate()
  {
  }
  virtual ~CbActivate()
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->requiresClient(this->lifecycleNodeClient_);

    lifecycleNodeClient_->onTransitionOnActivateSuccess_.connect([this]() { this->postSuccessEvent(); });
    lifecycleNodeClient_->onTransitionOnActivateFailure_.connect([this]() { this->postFailureEvent(); });
    lifecycleNodeClient_->onTransitionOnActivateError_.connect([this]() { this->postFailureEvent();});
  }

  virtual void onEntry() override
  {
    lifecycleNodeClient_->activate();
  }

private:
  ClLifecycleNode* lifecycleNodeClient_;
};
}  // namespace cl_lifecyclenode
