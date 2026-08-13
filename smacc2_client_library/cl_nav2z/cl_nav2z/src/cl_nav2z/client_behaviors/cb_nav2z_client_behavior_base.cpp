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
#include <cl_nav2z/client_behaviors/cb_nav2z_client_behavior_base.hpp>
#include <cl_nav2z/common.hpp>

namespace cl_nav2z
{
CbNav2ZClientBehaviorBase::~CbNav2ZClientBehaviorBase() {}

void CbNav2ZClientBehaviorBase::sendGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  if (!nav2ActionInterface_)
  {
    RCLCPP_ERROR(
      getLogger(), "[%s] Cannot send goal, CpNav2ActionInterface not available",
      getName().c_str());
    return;
  }

  if (!resultConnectionsInitialized_)
  {
    // Connect the action result signals so the behavior propagates results as
    // EvCbSuccess/EvCbFailure and records navigationResult_. Connection lifetime is
    // managed by the state machine (disconnected when this behavior is destroyed).
    this->onNavigationSucceeded(&CbNav2ZClientBehaviorBase::onNavigationActionSuccess, this);
    this->onNavigationAborted(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
    this->onNavigationCancelled(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
    resultConnectionsInitialized_ = true;
  }

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Sending goal");
  nav2ActionInterface_->sendGoal(goal);
}

void CbNav2ZClientBehaviorBase::onNavigationActionSuccess(
  const components::CpNav2ActionInterface::WrappedResult & r)
{
  navigationResult_ = r.code;
  RCLCPP_INFO(getLogger(), "[%s] Propagating success event from action server", getName().c_str());
  this->postSuccessEvent();
}

void CbNav2ZClientBehaviorBase::onNavigationActionAbort(
  const components::CpNav2ActionInterface::WrappedResult & r)
{
  navigationResult_ = r.code;
  RCLCPP_INFO(getLogger(), "[%s] Propagating failure event from action server", getName().c_str());
  this->postFailureEvent();
}

}  // namespace cl_nav2z
