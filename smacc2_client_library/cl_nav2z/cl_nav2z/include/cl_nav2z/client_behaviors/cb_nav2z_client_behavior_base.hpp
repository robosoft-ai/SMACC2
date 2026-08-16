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

    // Connect the action result signals here, on the state machine thread during
    // state configuration: connecting from the behavior's asynchronous onEntry
    // thread (as sendGoal used to) contends for the state machine mutex and can
    // deadlock against a concurrent state transition.
    if (!resultConnectionsInitialized_ && nav2ActionInterface_)
    {
      this->onNavigationSucceeded(&CbNav2ZClientBehaviorBase::onNavigationActionSuccess, this);
      this->onNavigationAborted(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
      this->onNavigationCancelled(&CbNav2ZClientBehaviorBase::onNavigationActionAbort, this);
      resultConnectionsInitialized_ = true;
    }

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

protected:
  // Sends the goal through CpNav2ActionInterface. The action result signals are
  // connected in onStateOrthogonalAllocation, so navigationResult_ is updated and
  // EvCbSuccess/EvCbFailure are posted when the navigation finishes.
  void sendGoal(nav2_msgs::action::NavigateToPose::Goal & goal);

  void cancelGoal()
  {
    if (nav2ActionInterface_)
    {
      nav2ActionInterface_->cancelNavigation();
    }
  }

  // Component-based signal connections
  template <typename T>
  smacc2::SmaccSignalConnection onNavigationSucceeded(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationSucceeded(callback, object);
    }
    return smacc2::SmaccSignalConnection();
  }

  template <typename T>
  smacc2::SmaccSignalConnection onNavigationAborted(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationAborted(callback, object);
    }
    return smacc2::SmaccSignalConnection();
  }

  template <typename T>
  smacc2::SmaccSignalConnection onNavigationCancelled(
    void (T::*callback)(const components::CpNav2ActionInterface::WrappedResult &), T * object)
  {
    if (nav2ActionInterface_)
    {
      return nav2ActionInterface_->onNavigationCancelled(callback, object);
    }
    return smacc2::SmaccSignalConnection();
  }

  // NEW: Component references instead of client reference
  components::CpNav2ActionInterface * nav2ActionInterface_ = nullptr;
  smacc2::client_core_components::CpActionClient<nav2_msgs::action::NavigateToPose> *
    actionClient_ = nullptr;

  rclcpp_action::ResultCode navigationResult_ = rclcpp_action::ResultCode::UNKNOWN;

  // Result handlers connected by sendGoal(). The base implementations store the
  // result code and post the behavior success/failure events; derived classes may
  // override to customize result handling (see CbNavigateNextWaypointUntilReached).
  virtual void onNavigationActionSuccess(const components::CpNav2ActionInterface::WrappedResult &);
  virtual void onNavigationActionAbort(const components::CpNav2ActionInterface::WrappedResult &);

private:
  bool resultConnectionsInitialized_ = false;
};

enum class SpinningPlanner
{
  Default,
  PureSpinning,
  Forward
};
}  // namespace cl_nav2z
