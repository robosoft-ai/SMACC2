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

#include <nitrosz_client/components/planner_switcher/cp_planner_switcher.hpp>
#include <nitrosz_client/nitrosz_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nitrosz
{
class CbNav2ZClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  virtual ~CbNav2ZClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(nitroszClient_);
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

protected:
  void sendGoal(ClNitrosZ::Goal & goal);

  void cancelGoal();

  // handling results according its type
  bool isOwnActionResponse(const ClNitrosZ::WrappedResult &);
  virtual void onNavigationResult(const ClNitrosZ::WrappedResult &);
  virtual void onNavigationActionSuccess(const ClNitrosZ::WrappedResult &);
  virtual void onNavigationActionAbort(const ClNitrosZ::WrappedResult &);

  cl_nitrosz::ClNitrosZ * nitroszClient_;
  cl_nitrosz::ClNitrosZ::SmaccNavigateResultSignal::SharedPtr navigationCallback_;

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
}  // namespace cl_nitrosz
