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

#include <smacc2/client_bases/smacc_action_client.hpp>
#include <smacc2/smacc_default_events.hpp>
#include <smacc2/smacc_signal.hpp>

#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>

namespace smacc2
{
namespace client_bases
{
using namespace smacc2::default_events;

template <typename ActionType>
class SmaccActionClientBase : public ISmaccActionClient
{
public:
  // Inside this macro you can find the typedefs for Goal and other types
  typedef rclcpp_action::Client<ActionType> ActionClient;

  using Goal = typename ActionClient::Goal;
  using Feedback = typename ActionClient::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  typedef typename GoalHandle::WrappedResult WrappedResult;

  using SendGoalOptions = typename ActionClient::SendGoalOptions;
  using GoalResponseCallback =
    std::function<void(std::shared_future<typename GoalHandle::SharedPtr>)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;
  using CancelRequest = typename ActionType::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionType::Impl::CancelGoalService::Response;
  using CancelCallback = std::function<void(typename CancelResponse::SharedPtr)>;

  typedef smacc2::SmaccSignal<void(const WrappedResult &)> SmaccActionResultSignal;

  std::string action_endpoint_;
  SmaccActionClientBase(std::string actionServerName) : ISmaccActionClient()
  {
    action_endpoint_ = actionServerName;
  }

  SmaccActionClientBase() : ISmaccActionClient() { name_ = ""; }

  virtual ~SmaccActionClientBase() {}

  virtual std::shared_ptr<rclcpp_action::ClientBase> getClientBase() override { return client_; }

  void onInitialize() override
  {
    if (name_ == "") name_ = smacc2::demangleSymbol(typeid(*this).name());
    this->client_ = rclcpp_action::create_client<ActionType>(getNode(), action_endpoint_);
    // RCLCPP_INFO_STREAM(
    //   this->getLogger(),
    //   "Waiting for action server '" << name_ << "' of type: " << demangledTypeName<ActionType>());
    //client_->wait_for_action_server();
  }

  void waitForServer()
  {
    RCLCPP_INFO_STREAM(
      this->getLogger(),
      "Waiting for action server '" << name_ << "' of type: " << demangledTypeName<ActionType>());
    client_->wait_for_action_server();
  }

  static std::string getEventLabel()
  {
    auto type = TypeInfo::getTypeInfoFromType<ActionType>();
    return type->getNonTemplatedTypeName();
  }

  std::optional<std::shared_future<typename GoalHandle::SharedPtr>> lastRequest_;
  std::optional<std::shared_future<typename CancelResponse::SharedPtr>> lastCancelResponse_;

  SmaccActionResultSignal onSucceeded_;
  SmaccActionResultSignal onAborted_;
  // SmaccActionResultSignal onPreempted_;
  // SmaccActionResultSignal onRejected_;
  SmaccActionResultSignal onCancelled_;

  // event creation/posting factory functions
  std::function<void(WrappedResult)> postSuccessEvent;
  std::function<void(WrappedResult)> postAbortedEvent;
  std::function<void(WrappedResult)> postCancelledEvent;
  // std::function<void(WrappedResult)> postPreemptedEvent;
  // std::function<void(WrappedResult)> postRejectedEvent;

  std::function<void(const Feedback &)> postFeedbackEvent;

  FeedbackCallback feedback_cb;

  template <typename EvType>
  void postResultEvent(WrappedResult & /*result*/)
  {
    auto * ev = new EvType();
    RCLCPP_INFO(
      getLogger(), "Action client Posting EVENT %s", demangleSymbol(typeid(ev).name()).c_str());
    this->postEvent(ev);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // we create here all the event factory functions capturing the TOrthogonal
    postSuccessEvent = [this](auto msg)
    { this->postResultEvent<EvActionSucceeded<TSourceObject, TOrthogonal>>(msg); };
    postAbortedEvent = [this](auto msg)
    { this->postResultEvent<EvActionAborted<TSourceObject, TOrthogonal>>(msg); };

    postCancelledEvent = [this](auto msg)
    { this->postResultEvent<EvActionCancelled<TSourceObject, TOrthogonal>>(msg); };

    postFeedbackEvent = [this](auto msg)
    {
      auto actionFeedbackEvent = new EvActionFeedback<Feedback, TOrthogonal>();
      actionFeedbackEvent->client = this;
      actionFeedbackEvent->feedbackMessage = msg;
      this->postEvent(actionFeedbackEvent);
      RCLCPP_DEBUG(getLogger(), "[%s] FEEDBACK EVENT", demangleType(typeid(*this)).c_str());
    };

    feedback_cb = [this](auto client, auto feedback) { this->onFeedback(client, feedback); };
  }

  template <typename T>
  smacc2::SmaccSignalConnection onSucceeded(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onSucceeded_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onSucceeded(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onSucceeded_, callback);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onAborted(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onAborted_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onAborted(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onAborted_, callback);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onCancelled(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onCancelled_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onCancelled(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onCancelled_, callback);
  }

  virtual bool cancelGoal() override
  {
    lastCancelResponse_ = this->client_->async_cancel_all_goals();

    return true;
  }

  std::shared_future<typename GoalHandle::SharedPtr> sendGoal(
    Goal & goal, typename SmaccActionResultSignal::WeakPtr resultCallback =
                   typename SmaccActionResultSignal::WeakPtr())
  {
    SendGoalOptions options;

    options.feedback_callback = feedback_cb;

    options.result_callback =
      [this, resultCallback](
        const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result)
    {
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "]  Action result callback, getting shared future");
      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "]  Action client Result goal id: "
                         << rclcpp_action::to_string(result.goal_id));

      // if (goalHandle_->get_goal_id() == result.goal_id)
      // {
      //   // goal_result_available_ = true;
      //   // result_ = result;
      //   RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "]  Result CB Goal id matches with last request");

      auto resultCallbackPtr = resultCallback.lock();

      if (resultCallbackPtr != nullptr)
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << getName() << "]  Result CB calling user callback:"
                           << demangleSymbol(typeid(*resultCallbackPtr).name()));
        (*resultCallbackPtr)(result);
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << getName() << "]  Result CB calling default callback");
        this->onResult(result);
      }
    };

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] client ready clients: "
                       << this->client_->get_number_of_ready_clients());
    RCLCPP_INFO_STREAM(
      getLogger(),
      "[" << getName() << "] Waiting it is ready? " << client_->action_server_is_ready());

    RCLCPP_INFO_STREAM(getLogger(), getName() << ": async send goal.");
    auto lastRequest = this->client_->async_send_goal(goal, options);
    this->lastRequest_ = lastRequest;

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Action request");

    return lastRequest;
  }

protected:
  typename ActionClient::SharedPtr client_;

  void onFeedback(
    typename GoalHandle::SharedPtr /*goalhandle*/,
    const std::shared_ptr<const Feedback> feedback_msg)
  {
    postFeedbackEvent(*feedback_msg);
  }

  void onResult(const WrappedResult & result_msg)
  {
    const auto & resultType = result_msg.code;

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] response result ["
                       << rclcpp_action::to_string(result_msg.goal_id) << "]: " << (int)resultType);

    if (resultType == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(getLogger(), "[%s] request result: Success", this->getName().c_str());
      onSucceeded_(result_msg);
      postSuccessEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_INFO(getLogger(), "[%s] request result: Aborted", this->getName().c_str());
      onAborted_(result_msg);
      postAbortedEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_INFO(getLogger(), "[%s] request result: Cancelled", this->getName().c_str());
      onCancelled_(result_msg);
      postCancelledEvent(result_msg);
    }
    else
    {
      RCLCPP_INFO(
        getLogger(), "[%s] request result: NOT HANDLED TYPE: %d", this->getName().c_str(),
        (int)resultType);
    }
  }
};

}  // namespace client_bases

}  // namespace smacc2
