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
  // ACTION_DEFINITION(ActionType);
  typedef rclcpp_action::Client<ActionType> ActionClient;
  // typedef actionlib::SimpleActionClient<ActionType> GoalHandle;

  // typedef typename ActionClient::SimpleDoneCallback SimpleDoneCallback;
  // typedef typename ActionClient::SimpleActiveCallback SimpleActiveCallback;
  // typedef typename ActionClient::SimpleFeedbackCallback SimpleFeedbackCallback;

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

  SmaccActionClientBase(std::string actionServerName)
  : ISmaccActionClient(), name_(actionServerName)
  {
  }

  SmaccActionClientBase() : ISmaccActionClient(), name_("") {}

  virtual ~SmaccActionClientBase() {}

  virtual std::shared_ptr<rclcpp_action::ClientBase> getClientBase() override { return client_; }

  void onInitialize() override
  {
    this->client_ = rclcpp_action::create_client<ActionType>(getNode(), name_);
    RCLCPP_INFO_STREAM(
      this->getNode()->get_logger(),
      "Waiting for action server '" << name_ << "' of type: " << demangledTypeName<ActionType>());
    client_->wait_for_action_server();
  }

  static std::string getEventLabel()
  {
    auto type = TypeInfo::getTypeInfoFromType<ActionType>();
    return type->getNonTemplatedTypeName();
  }

  /// rosnamespace path
  std::string name_;

  std::optional<std::shared_future<typename GoalHandle::SharedPtr>> lastRequest_;
  typename GoalHandle::SharedPtr goalHandle_;

  smacc2::SmaccSignal<void(const WrappedResult &)> onSucceeded_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onAborted_;
  // smacc2::SmaccSignal<void(const WrappedResult &)> onPreempted_;
  // smacc2::SmaccSignal<void(const WrappedResult &)> onRejected_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onCancelled_;

  // event creation/posting factory functions
  std::function<void(WrappedResult)> postSuccessEvent;
  std::function<void(WrappedResult)> postAbortedEvent;
  // std::function<void(WrappedResult)> postPreemptedEvent;
  // std::function<void(WrappedResult)> postRejectedEvent;
  std::function<void(WrappedResult)> postCancelledEvent;

  std::function<void(const Feedback &)> postFeedbackEvent;

  ResultCallback done_cb;
  // SimpleActiveCallback active_cb;
  FeedbackCallback feedback_cb;

  template <typename EvType>
  void postResultEvent(WrappedResult & /*result*/)
  {
    auto * ev = new EvType();
    // ev->client = this;
    // ev->resultMessage = *result;
    RCLCPP_INFO(
      getNode()->get_logger(), "Posting EVENT %s", demangleSymbol(typeid(ev).name()).c_str());
    this->postEvent(ev);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    // we create here all the event factory functions capturing the TOrthogonal
    postSuccessEvent = [=](auto msg) {
      this->postResultEvent<EvActionSucceeded<TSourceObject, TOrthogonal>>(msg);
    };
    postAbortedEvent = [=](auto msg) {
      this->postResultEvent<EvActionAborted<TSourceObject, TOrthogonal>>(msg);
    };
    // postPreemptedEvent = [=](auto msg) { this->postResultEvent<EvActionPreempted<TSourceObject, TOrthogonal>>(msg);
    // }; postRejectedEvent = [=](auto msg) { this->postResultEvent<EvActionRejected<TSourceObject, TOrthogonal>>(msg);
    // };
    postCancelledEvent = [=](auto msg) {
      this->postResultEvent<EvActionCancelled<TSourceObject, TOrthogonal>>(msg);
    };
    postFeedbackEvent = [=](auto msg) {
      auto actionFeedbackEvent = new EvActionFeedback<Feedback, TOrthogonal>();
      actionFeedbackEvent->client = this;
      actionFeedbackEvent->feedbackMessage = msg;
      this->postEvent(actionFeedbackEvent);
      RCLCPP_DEBUG(
        getNode()->get_logger(), "[%s] FEEDBACK EVENT", demangleType(typeid(*this)).c_str());
    };

    done_cb = [=](auto r) { this->onResult(r); };
    // done_cb = boost::bind(&SmaccActionClientBase<ActionType>::onResult, this, _1, _2);
    // active_cb;
    feedback_cb = [=](auto client, auto feedback) { this->onFeedback(client, feedback); };
  }

  template <typename T>
  boost::signals2::connection onSucceeded(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onSucceeded_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onSucceeded(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onSucceeded_, callback);
  }

  template <typename T>
  boost::signals2::connection onAborted(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onAborted_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onAborted(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onAborted_, callback);
  }

  template <typename T>
  boost::signals2::connection onCancelled(void (T::*callback)(WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onCancelled_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onCancelled(std::function<void(WrappedResult &)> callback)
  {
    return this->getStateMachine()->createSignalConnection(onCancelled_, callback);
  }

  /*
  template <typename T>
  boost::signals2::connection onPreempted(void (T::*callback)(WrappedResult &), T *object)
  {
      return this->getStateMachine()->createSignalConnection(onPreempted_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onPreempted(std::function<void(WrappedResult &)> callback)
  {
      return this->getStateMachine()->createSignalConnection(onPreempted_, callback);
  }

  template <typename T>
  boost::signals2::connection onRejected(void (T::*callback)(WrappedResult &), T *object)
  {
      return this->getStateMachine()->createSignalConnection(onRejected_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onRejected(std::function<void(WrappedResult &)> callback)
  {
      return this->getStateMachine()->createSignalConnection(onRejected_, callback);
  }
  */

  virtual void cancelGoal() override
  {
    if (lastRequest_ && lastRequest_->valid())
    {
      RCLCPP_INFO(getNode()->get_logger(), "Cancelling goal of %s", this->getName().c_str());
      std::shared_future<typename CancelResponse::SharedPtr> cancelresult =
        client_->async_cancel_goal(lastRequest_->get());

      // wait actively
      cancelresult.get();
      lastRequest_.reset();
    }
    else
    {
      RCLCPP_ERROR(
        getNode()->get_logger(),
        "%s [at %s]: not connected with actionserver, skipping cancel goal ...", getName().c_str(),
        getNamespace().c_str());
    }
  }

  /*
      virtual SimpleClientGoalState getState() override
      {
          return client_->getState();
      }*/

  void sendGoal(Goal & goal)
  {
    RCLCPP_INFO_STREAM(
      getNode()->get_logger(),
      "Sending goal with guid to actionserver located in " << this->name_ << "\"");

    // if (client_->isServerConnected())
    // {
    RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": Goal sent.");
    // client_->sendGoal(goal, done_cb, active_cb, feedback_cb);
    // std::shared_future<typename GoalHandle::SharedPtr>

    SendGoalOptions options;

    // GoalResponseCallback
    // options.goal_response_callback;

    /// Function called whenever feedback is received for the goal.
    // FeedbackCallback
    options.feedback_callback = feedback_cb;

    /// Function called when the result for the goal is received.
    // ResultCallback result_callback;
    // options.result_callback = done_cb;

    options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionType>::WrappedResult & result) {
        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        RCLCPP_INFO_STREAM(
          getNode()->get_logger(), getName() << ": Result callback, getting shared future");
        goalHandle_ = lastRequest_->get();
        RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": Result CB Check goal id");
        if (this->goalHandle_->get_goal_id() == result.goal_id)
        {
          // goal_result_available_ = true;
          // result_ = result;
          RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": Result CB Goal id matches");
          done_cb(result);
        }
        else
        {
          RCLCPP_INFO_STREAM(
            getNode()->get_logger(), getName() << ": Result CB Goal id DOES NOT match");
        }
      };

    // if (lastRequest_ && lastRequest_->valid())
    // {
    //   RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": checking previous request is really finished.");
    //   auto res = this->lastRequest_->get();
    //   RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": okay");
    // }
    // else
    // {
    //   RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": no previous request.");
    // }

    RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": async send goal.");
    this->lastRequest_ = this->client_->async_send_goal(goal, options);

    // RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": Goal Id: "  <<
    // rclcpp_action::to_string(lastRequest_->get()->get_goal_id()));

    RCLCPP_INFO_STREAM(
      getNode()->get_logger(),
      getName() << ": client ready clients: " << this->client_->get_number_of_ready_clients());
    RCLCPP_INFO_STREAM(
      getNode()->get_logger(), getName()
                                 << ": Waiting it is ready? " << client_->action_server_is_ready());
    // RCLCPP_INFO_STREAM(getNode()->get_logger(), getName() << ": spinning until completed");
    // if (rclcpp::spin_until_future_complete(this->getNode(), lastRequest_, std::chrono::seconds(2))
    // !=rclcpp::executor::FutureReturnCode::SUCCESS)
    // {
    //   throw std::runtime_error("send_goal failed");
    // }

    // goalHandle_ = lastRequest_->get();
    // if (!goalHandle_) {
    //   throw std::runtime_error("Goal was rejected by the action server");
    // }

    // }
    // else
    // {
    //     RCLCPP_ERROR(getNode()->get_logger(),"%s [at %s]: not connected with actionserver, skipping goal request
    //     ...", getName().c_str(), getNamespace().c_str());
    //     //client_->waitForServer();
    // }
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
    // auto *actionResultEvent = new EvActionResult<TDerived>();
    // actionResultEvent->client = this;
    // actionResultEvent->resultMessage = *result_msg;

    // const auto &resultType = this->getState();
    const auto & resultType = result_msg.code;

    RCLCPP_INFO_STREAM(
      getNode()->get_logger(), "[" << this->getName() << "] request result of request ["
                                   << rclcpp_action::to_string(result_msg.goal_id)
                                   << "]: " << (int)resultType);

    if (resultType == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(getNode()->get_logger(), "[%s] request result: Success", this->getName().c_str());
      onSucceeded_(result_msg);
      postSuccessEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_INFO(getNode()->get_logger(), "[%s] request result: Aborted", this->getName().c_str());
      onAborted_(result_msg);
      postAbortedEvent(result_msg);
    }
    else if (resultType == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_INFO(
        getNode()->get_logger(), "[%s] request result: Rejected", this->getName().c_str());
      onCancelled_(result_msg);
      postCancelledEvent(result_msg);
    }
    /*
    else if (resultType == actionlib::SimpleClientGoalState::REJECTED)
    {
        RCLCPP_INFO(getNode()->get_logger(),"[%s] request result: Rejected", this->getName().c_str());
        onRejected_(result_msg);
        postRejectedEvent(result_msg);
    }
    else if (resultType == actionlib::SimpleClientGoalState::PREEMPTED)
    {
        RCLCPP_INFO(getNode()->get_logger(),"[%s] request result: Preempted", this->getName().c_str());
        onPreempted_(result_msg);
        postPreemptedEvent(result_msg);
    }*/
    else
    {
      RCLCPP_INFO(
        getNode()->get_logger(), "[%s] request result: NOT HANDLED TYPE: %d",
        this->getName().c_str(), (int)resultType);
    }
  }
};

}  // namespace client_bases

}  // namespace smacc2
