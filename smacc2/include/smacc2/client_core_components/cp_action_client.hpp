// Copyright 2024 RobosoftAI Inc.
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

#pragma once

#include <smacc2/component.hpp>
#include <smacc2/smacc_default_events.hpp>
#include <smacc2/smacc_signal.hpp>
#include <smacc2/smacc_state_machine.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <mutex>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>

namespace smacc2
{
namespace client_core_components
{
using namespace smacc2::default_events;

template <typename ActionType>
class CpActionClient : public smacc2::ISmaccComponent
{
public:
  // Type aliases
  using ActionClient = rclcpp_action::Client<ActionType>;
  using Goal = typename ActionType::Goal;
  using Feedback = typename ActionType::Feedback;
  using Result = typename ActionType::Result;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using SendGoalOptions = typename ActionClient::SendGoalOptions;
  using GoalResponseCallback =
    std::function<void(std::shared_future<typename GoalHandle::SharedPtr>)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;

  // Configuration options
  std::optional<std::string> actionServerName;
  std::optional<std::chrono::milliseconds> serverTimeout;

  // SMACC2 Signals for component communication
  smacc2::SmaccSignal<void(const WrappedResult &)> onActionSucceeded_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onActionAborted_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onActionCancelled_;
  smacc2::SmaccSignal<void(const Feedback &)> onActionFeedback_;
  smacc2::SmaccSignal<void()> onGoalAccepted_;
  smacc2::SmaccSignal<void()> onGoalRejected_;

  // Event posting functions (set during orthogonal allocation)
  std::function<void(const WrappedResult &)> postSuccessEvent;
  std::function<void(const WrappedResult &)> postAbortedEvent;
  std::function<void(const WrappedResult &)> postCancelledEvent;
  std::function<void(const Feedback &)> postFeedbackEvent;

  // Constructor
  CpActionClient() = default;

  CpActionClient(const std::string & actionServerName) : actionServerName(actionServerName) {}

  virtual ~CpActionClient() = default;

  // Public API
  std::shared_future<typename GoalHandle::SharedPtr> sendGoal(
    Goal & goal, typename smacc2::SmaccSignal<void(const WrappedResult &)>::WeakPtr resultCallback =
                   typename smacc2::SmaccSignal<void(const WrappedResult &)>::WeakPtr())
  {
    std::lock_guard<std::mutex> lock(actionMutex_);

    SendGoalOptions options;

    // Set up feedback callback
    options.feedback_callback = feedbackCallback_;

    // Set up result callback
    options.result_callback = [this, resultCallback](const WrappedResult & result)
    {
      std::lock_guard<std::mutex> lock(actionMutex_);

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << this->getName() << "] Action result callback, goal id: "
                         << rclcpp_action::to_string(result.goal_id));

      auto resultCallbackPtr = resultCallback.lock();
      if (resultCallbackPtr != nullptr)
      {
        RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Calling user result callback");
        (*resultCallbackPtr)(result);
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName() << "] Using default result handling");
        this->onResult(result);
      }
    };

    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << this->getName() << "] Sending goal to action server: " << (long)client_.get());

    auto goalFuture = client_->async_send_goal(goal, options);
    lastRequest_ = goalFuture;

    return goalFuture;
  }

  bool cancelGoal()
  {
    std::lock_guard<std::mutex> lock(actionMutex_);

    if (lastRequest_ && lastRequest_->valid())
    {
      RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Cancelling current goal");

      auto cancelFuture = client_->async_cancel_all_goals();
      lastCancelResponse_ = cancelFuture;
      return true;
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "[" << this->getName() << "] No active goal to cancel");
      return false;
    }
  }

  bool isServerReady() const { return client_ && client_->action_server_is_ready(); }

  void waitForServer()
  {
    if (client_)
    {
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << this->getName() << "] Waiting for action server: " << *actionServerName);
      client_->wait_for_action_server();
    }
  }

  // Component lifecycle
  void onInitialize() override
  {
    if (!actionServerName)
    {
      RCLCPP_ERROR_STREAM(getLogger(), "[" << this->getName() << "] Action server name not set!");
      return;
    }

    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << this->getName() << "] Initializing action client for: " << *actionServerName);

    client_ = rclcpp_action::create_client<ActionType>(getNode(), *actionServerName);
    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << this->getName() << "] DONE Initializing action client for: " << *actionServerName);

    // Set up feedback callback
    feedbackCallback_ = [this](auto goalHandle, auto feedback)
    { this->onFeedback(goalHandle, feedback); };
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onComponentInitialization()
  {
    // Set up event posting functions with proper template parameters
    postSuccessEvent = [this](const WrappedResult & result)
    { this->postResultEvent<EvActionSucceeded<TSourceObject, TOrthogonal>>(result); };

    postAbortedEvent = [this](const WrappedResult & result)
    { this->postResultEvent<EvActionAborted<TSourceObject, TOrthogonal>>(result); };

    postCancelledEvent = [this](const WrappedResult & result)
    { this->postResultEvent<EvActionCancelled<TSourceObject, TOrthogonal>>(result); };

    postFeedbackEvent = [this](const Feedback & feedback)
    {
      auto actionFeedbackEvent = new EvActionFeedback<Feedback, TOrthogonal>();
      // actionFeedbackEvent->client = this->owner_;
      actionFeedbackEvent->feedbackMessage = feedback;
      this->postEvent(actionFeedbackEvent);
      RCLCPP_DEBUG(getLogger(), "[%s] FEEDBACK EVENT", this->getName().c_str());
    };
  }

  // Signal connection methods
  template <typename T>
  boost::signals2::connection onSucceeded(void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onActionSucceeded_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onAborted(void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onActionAborted_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onCancelled(void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onActionCancelled_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onFeedback(void (T::*callback)(const Feedback &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onActionFeedback_, callback, object);
  }

  // Access to underlying client for advanced usage
  std::shared_ptr<ActionClient> getActionClient() const { return client_; }

private:
  std::shared_ptr<ActionClient> client_ = nullptr;
  std::optional<std::shared_future<typename GoalHandle::SharedPtr>> lastRequest_;
  std::optional<
    std::shared_future<typename rclcpp_action::Client<ActionType>::CancelResponse::SharedPtr>>
    lastCancelResponse_;
  FeedbackCallback feedbackCallback_;
  std::mutex actionMutex_;

  void onFeedback(
    typename GoalHandle::SharedPtr /* goalHandle */,
    const std::shared_ptr<const Feedback> feedback_msg)
  {
    onActionFeedback_(*feedback_msg);
    postFeedbackEvent(*feedback_msg);
  }

  void onResult(const WrappedResult & result_msg)
  {
    const auto & resultType = result_msg.code;

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Action result ["
                       << rclcpp_action::to_string(result_msg.goal_id) << "]: " << (int)resultType);

    if (resultType == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(getLogger(), "[%s] Action result: Success", this->getName().c_str());
      onActionSucceeded_(result_msg);
      postSuccessEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_INFO(getLogger(), "[%s] Action result: Aborted", this->getName().c_str());
      onActionAborted_(result_msg);
      postAbortedEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_INFO(getLogger(), "[%s] Action result: Cancelled", this->getName().c_str());
      onActionCancelled_(result_msg);
      postCancelledEvent(result_msg);
    }
    else
    {
      RCLCPP_WARN(
        getLogger(), "[%s] Action result: Unhandled type: %d", this->getName().c_str(),
        (int)resultType);
    }
  }

  template <typename EvType>
  void postResultEvent(const WrappedResult & /* result */)
  {
    auto * ev = new EvType();
    RCLCPP_INFO(
      getLogger(), "[%s] Posting event: %s", this->getName().c_str(),
      smacc2::demangleSymbol(typeid(ev).name()).c_str());
    this->postEvent(ev);
  }
};

}  // namespace client_core_components
}  // namespace smacc2
