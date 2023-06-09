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
#include <nav2z_client/client_behaviors/cb_nav2z_client_behavior_base.hpp>
#include <nav2z_client/common.hpp>
namespace cl_nav2z
{
CbNav2ZClientBehaviorBase::~CbNav2ZClientBehaviorBase() {}

void CbNav2ZClientBehaviorBase::sendGoal(ClNav2Z::Goal & goal)
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Creating sending goal callback signal.");

  this->navigationCallback_ = std::make_shared<cl_nav2z::ClNav2Z::SmaccNavigateResultSignal>();

  this->getStateMachine()->createSignalConnection(
    *navigationCallback_, &CbNav2ZClientBehaviorBase::onNavigationResult, this);

  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Sending goal");
  goalHandleFuture_ = this->nav2zClient_->sendGoal(goal, navigationCallback_);
  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] Goal set, future valid: " << goalHandleFuture_.valid());

  // this->goal_uuid_ = gh.get_goal_id () ;
}

void CbNav2ZClientBehaviorBase::cancelGoal() { this->nav2zClient_->cancelGoal(); }

bool CbNav2ZClientBehaviorBase::isOwnActionResponse(const ClNav2Z::WrappedResult & r)
{
  auto name = getName();
  if (isShutdownRequested())
  {
    RCLCPP_ERROR(
      getLogger(), "[%s] Propagating action client signal skipped. the behavior is shutting down",
      name.c_str());
    return false;
  }

  if (!goalHandleFuture_.valid())
  {
    RCLCPP_ERROR(
      this->getLogger(), "[%s]Propagating action client signal, our goal handle is not valid",
      name.c_str());
    return false;
  }

  auto goalHandle = goalHandleFuture_.get();
  auto goal_uuid_ = goalHandle->get_goal_id();

  if (r.goal_id != goal_uuid_)
  {
    RCLCPP_ERROR(
      getLogger(),
      "[%s] Received a failure event from an action server with a different goal_uuid: %s, "
      "expected: %s",
      smacc2::demangleType(typeid(*this)).c_str(), rclcpp_action::to_string(r.goal_id).c_str(),
      rclcpp_action::to_string(goal_uuid_).c_str());
    return false;
  }

  return true;
}

void CbNav2ZClientBehaviorBase::onNavigationResult(const ClNav2Z::WrappedResult & r)
{
  if (r.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    this->onNavigationActionSuccess(r);
  }
  else
  {
    this->onNavigationActionAbort(r);
  }
}

void CbNav2ZClientBehaviorBase::onNavigationActionSuccess(const ClNav2Z::WrappedResult & r)
{
  // if (!isOwnActionResponse(r))
  // {
  //   RCLCPP_WARN(
  //     getLogger(), "[%s] Propagating success event skipped. Action response is not ours.",
  //     getName().c_str());
  //   return;
  // }

  navigationResult_ = r.code;

  RCLCPP_INFO(getLogger(), "[%s] Propagating success event from action server", getName().c_str());
  this->postSuccessEvent();
}

void CbNav2ZClientBehaviorBase::onNavigationActionAbort(const ClNav2Z::WrappedResult & r)
{
  // if (!isOwnActionResponse(r))
  // {
  //   RCLCPP_WARN(
  //     getLogger(), "[%s] Propagating success event skipped. Action response is not ours.",
  //     getName().c_str());
  //   return;
  // }

  navigationResult_ = r.code;
  auto name = getName();
  RCLCPP_INFO(getLogger(), "[%s] Propagating failure event from action server", name.c_str());
  this->postFailureEvent();
}
}  // namespace cl_nav2z
