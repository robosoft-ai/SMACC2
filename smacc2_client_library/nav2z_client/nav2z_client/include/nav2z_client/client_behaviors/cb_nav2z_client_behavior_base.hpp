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
    this->requiresClient(nav2zClient_);
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    // nav2zClient_->onSucceeded(&CbNav2ZClientBehaviorBase::onNavigationActionSuccess, this);
    // nav2zClient_->onAborted(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
    // nav2zClient_->onCancelled(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
  }

protected:
  void sendGoal(ClNav2Z::Goal & goal);

  void cancelGoal();

  bool isOwnActionResponse(smacc2::SmaccSignal<void (const ClNav2Z::WrappedResult &)> & r);

  virtual void onNavigationResult(smacc2::SmaccSignal<void (const ClNav2Z::WrappedResult &)> & r);

  virtual void onNavigationActionSuccess(smacc2::SmaccSignal<void (const ClNav2Z::WrappedResult &)> &);
  virtual void onNavigationActionAbort(smacc2::SmaccSignal<void (const ClNav2Z::WrappedResult &)> &);

  cl_nav2z::ClNav2Z * nav2zClient_;

  rclcpp_action::ResultCode navigationResult_;

  std::shared_future<
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose> > >
    goalHandleFuture_;
};

enum class SpinningPlanner
{
  Default,
  PureSpinning,
  Forward
};
}  // namespace cl_nav2z
