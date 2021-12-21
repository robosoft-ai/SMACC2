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

#include <nav2z_client/components/planner_switcher/planner_switcher.hpp>
#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
class CbNav2ZClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  virtual ~CbNav2ZClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    moveBaseClient_->onSucceeded(&CbNav2ZClientBehaviorBase::propagateSuccessEvent, this);
    moveBaseClient_->onAborted(&CbNav2ZClientBehaviorBase::propagateFailureEvent, this);
    moveBaseClient_->onCancelled(&CbNav2ZClientBehaviorBase::propagateFailureEvent, this);
  }

protected:
  cl_nav2z::ClNav2Z * moveBaseClient_;

  rclcpp_action::ResultCode navigationResult_;

private:
  void propagateSuccessEvent(ClNav2Z::WrappedResult &);
  void propagateFailureEvent(ClNav2Z::WrappedResult &);
};

enum class SpinningPlanner
{
  Default,
  PureSpinning,
  Forward
};
}  // namespace cl_nav2z
