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

#include <nav2z_client/components/planner_switcher/cp_planner_switcher.hpp>
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
  }

protected:
  void sendGoal(ClNav2Z::Goal & goal);

  void cancelGoal();

  // handling results according its type
  bool isOwnActionResponse(const ClNav2Z::WrappedResult &);
  virtual void onNavigationResult(const ClNav2Z::WrappedResult &);
  virtual void onNavigationActionSuccess(const ClNav2Z::WrappedResult &);
  virtual void onNavigationActionAbort(const ClNav2Z::WrappedResult &);

  cl_nav2z::ClNav2Z * nav2zClient_;
  cl_nav2z::ClNav2Z::SmaccNavigateResultSignal::SharedPtr navigationCallback_;

  // deprecated
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
