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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{
using namespace smacc2;
class CbNav2ZClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  virtual ~CbNav2ZClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // NEW: Pure component-based approach - no client dependencies
    this->requiresComponent(nav2ActionInterface_, ComponentRequirement::HARD);
    this->requiresComponent(actionClient_, ComponentRequirement::HARD);

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  // LEGACY COMPATIBILITY: For third-party code that still calls the old method
  template <typename TOrthogonal, typename TSourceObject>
  [[deprecated(
    "Use onStateOrthogonalAllocation instead. This method exists only for third-party "
    "compatibility.")]] void
  onOrthogonalAllocation()
  {
    onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

protected:
  // NEW: Component-based API - uses components directly
  void sendGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
  {
    if (nav2ActionInterface_)
    {
      nav2ActionInterface_->sendGoal(goal);
    }
  }

  void cancelGoal()
  {
    if (nav2ActionInterface_)
    {
      nav2ActionInterface_->cancelNavigation();
    }
  }

  // Component-based signal connections
  template <typename T>
  boost::signals2::connection onNavigationSucceeded(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationSucceeded(callback, object);
    }
    return boost::signals2::connection();
  }

  template <typename T>
  boost::signals2::connection onNavigationAborted(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationAborted(callback, object);
    }
    return boost::signals2::connection();
  }

  template <typename T>
  boost::signals2::connection onNavigationCancelled(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationCancelled(callback, object);
    }
    return boost::signals2::connection();
  }

  // NEW: Component references instead of client reference
  components::CpNav2ActionInterface * nav2ActionInterface_ = nullptr;
  smacc2::client_core_components::CpActionClient<nav2_msgs::action::NavigateToPose> *
    actionClient_ = nullptr;

  // Common legacy member for result tracking
  rclcpp_action::ResultCode navigationResult_;

  // REMOVED: All legacy client-based members
  // ❌ cl_nav2z::ClNav2Z * nav2zClient_
  // ❌ cl_nav2z::ClNav2Z::SmaccNavigateResultSignal::SharedPtr navigationCallback_
  // ❌ rclcpp_action::ResultCode navigationResult_
  // ❌ goalHandleFuture_

  // Virtual methods for derived classes - now use component types
  virtual void onNavigationResult(const components::CpNav2ActionInterface::WrappedResult &) {}
  virtual void onNavigationActionSuccess(const components::CpNav2ActionInterface::WrappedResult &)
  {
  }
  virtual void onNavigationActionAbort(const components::CpNav2ActionInterface::WrappedResult &) {}
};

enum class SpinningPlanner
{
  Default,
  PureSpinning,
  Forward
};
}  // namespace cl_nav2z
