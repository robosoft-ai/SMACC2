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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

#include <functional>
#include <future>

namespace cl_nav2z
{
namespace components
{
using namespace smacc2;

class CpNav2ActionInterface : public smacc2::ISmaccComponent
{
public:
  // Type aliases for convenience
  using ActionType = nav2_msgs::action::NavigateToPose;
  using Goal = ActionType::Goal;
  using Result = ActionType::Result;
  using Feedback = ActionType::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using ActionClient = smacc2::client_core_components::CpActionClient<ActionType>;

  // Nav2-specific signals for external subscribers
  smacc2::SmaccSignal<void(const WrappedResult &)> onNavigationSucceeded_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onNavigationAborted_;
  smacc2::SmaccSignal<void(const WrappedResult &)> onNavigationCancelled_;
  smacc2::SmaccSignal<void(const Feedback &)> onNavigationFeedback_;

  // Nav2-specific event posting functions (set during orthogonal allocation)
  std::function<void(const WrappedResult &)> postNavigationSuccessEvent;
  std::function<void(const WrappedResult &)> postNavigationAbortedEvent;
  std::function<void(const WrappedResult &)> postNavigationCancelledEvent;
  std::function<void(const Feedback &)> postNavigationFeedbackEvent;

  CpNav2ActionInterface() = default;
  virtual ~CpNav2ActionInterface() = default;

  // Public API for behaviors and other components
  std::shared_future<typename GoalHandle::SharedPtr> sendNavigationGoal(
    const geometry_msgs::msg::PoseStamped & target)
  {
    if (!actionClient_)
    {
      RCLCPP_ERROR(getLogger(), "[CpNav2ActionInterface] Action client not available!");
      throw std::runtime_error("Action client not initialized");
    }

    Goal goal;
    goal.pose = target;

    // Set default behavior_tree if not specified
    if (goal.behavior_tree.empty())
    {
      goal.behavior_tree = "";  // Let Nav2 use default behavior tree
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[CpNav2ActionInterface] Sending navigation goal to: "
                     << "x=" << target.pose.position.x << ", y=" << target.pose.position.y
                     << ", frame=" << target.header.frame_id);

    return actionClient_->sendGoal(goal);
  }

  std::shared_future<typename GoalHandle::SharedPtr> sendGoal(Goal & goal)
  {
    if (!actionClient_)
    {
      RCLCPP_ERROR(getLogger(), "[CpNav2ActionInterface] Action client not available!");
      throw std::runtime_error("Action client not initialized");
    }

    return actionClient_->sendGoal(goal);
  }

  bool cancelNavigation()
  {
    if (!actionClient_)
    {
      RCLCPP_WARN(getLogger(), "[CpNav2ActionInterface] Action client not available for cancel!");
      return false;
    }

    RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Cancelling navigation goal");
    return actionClient_->cancelGoal();
  }

  bool isNavigationServerReady() const { return actionClient_ && actionClient_->isServerReady(); }

  void waitForNavigationServer()
  {
    if (actionClient_)
    {
      actionClient_->waitForServer();
    }
  }

  // Component lifecycle
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Set up nav2-specific event posting functions
    postNavigationSuccessEvent = [this](const WrappedResult & result)
    {
      auto * ev = new smacc2::default_events::EvActionSucceeded<TClient, TOrthogonal>();
      this->postEvent(ev);
    };

    postNavigationAbortedEvent = [this](const WrappedResult & result)
    {
      auto * ev = new smacc2::default_events::EvActionAborted<TClient, TOrthogonal>();
      this->postEvent(ev);
    };

    postNavigationCancelledEvent = [this](const WrappedResult & result)
    {
      auto * ev = new smacc2::default_events::EvActionCancelled<TClient, TOrthogonal>();
      this->postEvent(ev);
    };

    postNavigationFeedbackEvent = [this](const Feedback & feedback)
    {
      auto * ev = new smacc2::default_events::EvActionFeedback<TClient, TOrthogonal>();
      //ev->feedbackMessage = feedback;
      this->postEvent(ev);
    };

    RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Component initialized");
  }

  void onInitialize() override
  {
    RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Component initializing");

    // Require the underlying action client component
    this->requiresComponent(actionClient_, ComponentRequirement::HARD);

    // Wire up signal connections from action client to nav2-specific signals
    if (actionClient_)
    {
      // Connect action client signals to our nav2-specific signals and event posting using method pointers
      // these are the main smacc signals in the action client, our navigation client behavior subscribes to them and reacts accordingly
      // each derived behavior do something different in reaction to these signals
      actionClient_->onSucceeded(&CpNav2ActionInterface::onNavigationSuccessCallback, this);
      actionClient_->onAborted(&CpNav2ActionInterface::onNavigationAbortedCallback, this);
      actionClient_->onCancelled(&CpNav2ActionInterface::onNavigationCancelledCallback, this);
      actionClient_->onFeedback(&CpNav2ActionInterface::onNavigationFeedbackCallback, this);

      RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Signal connections established");
    }
    else
    {
      RCLCPP_ERROR(
        getLogger(), "[CpNav2ActionInterface] Action client not found during initialization!");
    }
  }

  // Access to underlying action client for advanced usage
  ActionClient * getActionClient() const { return actionClient_; }

  // These methods are used by other client behavior and components that want to react to navigation events.
  template <typename T>
  boost::signals2::connection onNavigationSucceeded(
    void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(
      onNavigationSucceeded_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onNavigationAborted(
    void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onNavigationAborted_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onNavigationCancelled(
    void (T::*callback)(const WrappedResult &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(
      onNavigationCancelled_, callback, object);
  }

  template <typename T>
  boost::signals2::connection onNavigationFeedback(
    void (T::*callback)(const Feedback &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onNavigationFeedback_, callback, object);
  }

private:
  smacc2::client_core_components::CpActionClient<ActionType> * actionClient_ = nullptr;

  // Event translation callbacks
  void onNavigationSuccessCallback(const WrappedResult & result)
  {
    RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Navigation succeeded");
    onNavigationSucceeded_(result);
    if (postNavigationSuccessEvent)
    {
      postNavigationSuccessEvent(result);
    }
  }

  void onNavigationAbortedCallback(const WrappedResult & result)
  {
    RCLCPP_WARN(getLogger(), "[CpNav2ActionInterface] Navigation aborted");
    onNavigationAborted_(result);
    if (postNavigationAbortedEvent)
    {
      postNavigationAbortedEvent(result);
    }
  }

  void onNavigationCancelledCallback(const WrappedResult & result)
  {
    RCLCPP_INFO(getLogger(), "[CpNav2ActionInterface] Navigation cancelled");
    onNavigationCancelled_(result);
    if (postNavigationCancelledEvent)
    {
      postNavigationCancelledEvent(result);
    }
  }

  void onNavigationFeedbackCallback(const Feedback & feedback)
  {
    RCLCPP_DEBUG(getLogger(), "[CpNav2ActionInterface] Navigation feedback received");
    onNavigationFeedback_(feedback);
    if (postNavigationFeedbackEvent)
    {
      postNavigationFeedbackEvent(feedback);
    }
  }
};

}  // namespace components
}  // namespace cl_nav2z
