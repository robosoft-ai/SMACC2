// Copyright 2026 RobosoftAI Inc.
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
 * 	 Authors: Brett Aldrich, Pablo Inigo Blasco
 *
 ******************************************************************************************************************/

#pragma once

#include <atomic>
#include <chrono>

#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behavior_bases
{

// Generic base for client behaviors that send one goal to a ROS 2 action server
// (through a CpActionClient<TAction> component) and finish when the action
// finishes. Derived behaviors fill a Goal in onEntry() and call sendGoal();
// the base turns the action result into the state-scoped behavior events
// EvCbSuccess<TDerived, TOrthogonal> / EvCbFailure<TDerived, TOrthogonal>.
//
// Lifecycle contract (see the async-thread locking rule in CLAUDE.md):
// - components are resolved and result signals wired in
//   onStateOrthogonalAllocation, on the state machine thread. Never from the
//   asynchronous onEntry/onExit threads.
// - a derived class that declares its own onStateOrthogonalAllocation MUST
//   chain to this one (and this one chains to SmaccAsyncClientBehavior's, which
//   installs the event-posting functions - skipping the chain leaves them empty
//   and postSuccessEvent() throws std::bad_function_call).
// - no machine-scoped events are posted by this base: transition tables react
//   to the EvCb* behavior events, which cannot leak across state transitions.
template <typename TAction>
class CbActionClientBehaviorBase : public smacc2::SmaccAsyncClientBehavior
{
public:
  using Goal = typename TAction::Goal;
  using GoalHandle = rclcpp_action::ClientGoalHandle<TAction>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using Feedback = typename TAction::Feedback;
  using ActionClientComponent = smacc2::client_core_components::CpActionClient<TAction>;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(actionClient_, ComponentRequirement::HARD);

    if (!resultConnectionsInitialized_ && actionClient_ != nullptr)
    {
      actionClient_->onSucceeded(&CbActionClientBehaviorBase::onActionSuccess, this);
      actionClient_->onAborted(&CbActionClientBehaviorBase::onActionAbort, this);
      actionClient_->onCancelled(&CbActionClientBehaviorBase::onActionAbort, this);
      actionClient_->onFeedback(&CbActionClientBehaviorBase::onActionFeedback, this);
      resultConnectionsInitialized_ = true;
    }

    smacc2::SmaccAsyncClientBehavior::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  virtual ~CbActionClientBehaviorBase() {}

  // If the state exits while a goal is still in flight (e.g. a keyboard or
  // timeout transition), cancel it: unlike bt_navigator navigation - where the
  // next goal implicitly preempts - each behavior/docking server action keeps
  // executing an abandoned goal, leaving the robot moving under a command
  // nobody owns. Derived classes overriding onExit must chain to this.
  void onExit() override
  {
    if (goalInFlight_)
    {
      RCLCPP_WARN(
        getLogger(), "[%s] State exited with the action goal still in flight - cancelling",
        getName().c_str());
      cancelGoal();
    }
  }

protected:
  // Sends the goal and waits (in the calling asynchronous onEntry thread) for the
  // server's goal response. Returns true if the goal was accepted. On a
  // not-ready server, a rejected goal, or a response timeout it posts the
  // failure event and returns false - the caller can simply return from
  // onEntry(). This closes the silent-hang gap of a rejected/unanswered goal:
  // CpActionClient installs no goal_response handling, and a rejected goal
  // never invokes the result callback.
  bool sendGoal(Goal & goal)
  {
    if (actionClient_ == nullptr || !actionClient_->isServerReady())
    {
      RCLCPP_ERROR(
        getLogger(), "[%s] Action server not available, cannot send goal", getName().c_str());
      this->postFailureEvent();
      return false;
    }

    auto goalHandleFuture = actionClient_->sendGoal(goal);

    // wait for goal acceptance in short slices so a state exit is honored
    auto deadline = std::chrono::steady_clock::now() + goalResponseTimeout_;
    while (!this->isShutdownRequested() && std::chrono::steady_clock::now() < deadline)
    {
      if (goalHandleFuture.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
      {
        if (goalHandleFuture.get() != nullptr)
        {
          goalInFlight_ = true;  // accepted; the result signals will finish the behavior
          return true;
        }

        RCLCPP_ERROR(getLogger(), "[%s] Goal was rejected by the action server", getName().c_str());
        this->postFailureEvent();
        return false;
      }
    }

    if (!this->isShutdownRequested())
    {
      RCLCPP_ERROR(
        getLogger(), "[%s] Timed out waiting for the action server goal response",
        getName().c_str());
      this->postFailureEvent();
    }
    return false;
  }

  void cancelGoal()
  {
    if (actionClient_ != nullptr)
    {
      actionClient_->cancelGoal();
    }
  }

  // Result handlers connected by onStateOrthogonalAllocation. Base
  // implementations record the result code and post the behavior events;
  // derived classes may override to customize result handling.
  virtual void onActionSuccess(const WrappedResult & result)
  {
    goalInFlight_ = false;
    actionResult_ = result.code;
    RCLCPP_INFO(getLogger(), "[%s] Action succeeded, propagating success event", getName().c_str());
    this->postSuccessEvent();
  }

  virtual void onActionAbort(const WrappedResult & result)
  {
    goalInFlight_ = false;
    actionResult_ = result.code;
    RCLCPP_INFO(getLogger(), "[%s] Action failed, propagating failure event", getName().c_str());
    this->postFailureEvent();
  }

  // optional: override to consume action feedback (distance traveled etc.)
  virtual void onActionFeedback(const Feedback & /*feedback*/) {}

  ActionClientComponent * actionClient_ = nullptr;

  rclcpp_action::ResultCode actionResult_ = rclcpp_action::ResultCode::UNKNOWN;

  // how long sendGoal waits for the server to accept/reject the goal
  std::chrono::milliseconds goalResponseTimeout_ = std::chrono::milliseconds(10000);

private:
  bool resultConnectionsInitialized_ = false;
  std::atomic<bool> goalInFlight_{false};
};

}  // namespace client_behavior_bases
}  // namespace smacc2
