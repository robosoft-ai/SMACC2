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

#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.hpp>
#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_move_base_z
{
class CbMoveBaseClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  virtual ~CbMoveBaseClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    moveBaseClient_->onSucceeded(&CbMoveBaseClientBehaviorBase::propagateSuccessEvent, this);
    moveBaseClient_->onAborted(&CbMoveBaseClientBehaviorBase::propagateFailureEvent, this);
  }

protected:
  cl_move_base_z::ClMoveBaseZ * moveBaseClient_;

private:
  void propagateSuccessEvent(ClMoveBaseZ::WrappedResult &);
  void propagateFailureEvent(ClMoveBaseZ::WrappedResult &);
};
}  // namespace cl_move_base_z
