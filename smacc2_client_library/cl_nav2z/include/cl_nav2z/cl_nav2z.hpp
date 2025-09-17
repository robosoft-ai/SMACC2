// Copyright 2024 Robosoft Inc.
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

#include <cl_nav2z/components/cp_nav2_action_interface.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc.hpp>

namespace cl_nav2z
{

// Refactored ClNav2Z using pure component composition
class ClNav2Z : public smacc2::ISmaccClient
{
public:
  // Type aliases for backward compatibility with existing behaviors
  using ActionType = nav2_msgs::action::NavigateToPose;
  using Goal = ActionType::Goal;
  using Result = ActionType::Result;
  using Feedback = ActionType::Feedback;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  // Legacy type alias for backward compatibility
  typedef smacc2::SmaccSignal<void(const WrappedResult &)> SmaccNavigateResultSignal;

  // Constructor
  ClNav2Z(std::string actionServerName = "/navigate_to_pose") : actionServerName_(actionServerName)
  {
  }

  virtual ~ClNav2Z() = default;

  // Component composition during orthogonal initialization
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Create core action client component
    auto actionClient = this->createComponent<
      smacc2::client_core_components::CpActionClient<ActionType>, TOrthogonal, ClNav2Z>();
    actionClient->actionServerName = actionServerName_;

    // Create nav2-specific interface component (requires actionClient)
    this->createComponent<components::CpNav2ActionInterface, TOrthogonal, ClNav2Z>();

    RCLCPP_INFO_STREAM(
      this->getLogger(), "[ClNav2Z] Components created for action server: " << actionServerName_);
  }

  // Public API for backward compatibility with existing behaviors
  std::shared_future<typename GoalHandle::SharedPtr> sendGoal(Goal & goal)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->sendGoal(goal);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Nav2 interface component not available!");
      throw std::runtime_error("Nav2 interface component not initialized");
    }
  }

  // Overload for compatibility with existing behaviors that pass callbacks
  std::shared_future<typename GoalHandle::SharedPtr> sendGoal(
    Goal & goal, typename SmaccNavigateResultSignal::WeakPtr resultCallback)
  {
    auto actionClient = getActionClient();
    if (actionClient)
    {
      return actionClient->sendGoal(goal, resultCallback);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Action client component not available!");
      throw std::runtime_error("Action client component not initialized");
    }
  }

  // Convenience method for pose-based navigation
  std::shared_future<typename GoalHandle::SharedPtr> sendNavigationGoal(
    const geometry_msgs::msg::PoseStamped & target)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->sendNavigationGoal(target);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Nav2 interface component not available!");
      throw std::runtime_error("Nav2 interface component not initialized");
    }
  }

  bool cancelGoal()
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->cancelNavigation();
    }
    else
    {
      RCLCPP_WARN(getLogger(), "[ClNav2Z] Nav2 interface component not available for cancel!");
      return false;
    }
  }

  bool isServerReady()
  {
    auto navInterface = getNavInterface();
    return navInterface && navInterface->isNavigationServerReady();
  }

  void waitForServer()
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      navInterface->waitForNavigationServer();
    }
  }

  // Signal connection methods for backward compatibility
  template <typename T>
  boost::signals2::connection onSucceeded(void (T::*callback)(const WrappedResult &), T * object)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->onNavigationSucceeded(callback, object);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Cannot connect signal - Nav2 interface not available!");
      return boost::signals2::connection();
    }
  }

  template <typename T>
  boost::signals2::connection onAborted(void (T::*callback)(const WrappedResult &), T * object)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->onNavigationAborted(callback, object);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Cannot connect signal - Nav2 interface not available!");
      return boost::signals2::connection();
    }
  }

  template <typename T>
  boost::signals2::connection onCancelled(void (T::*callback)(const WrappedResult &), T * object)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->onNavigationCancelled(callback, object);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Cannot connect signal - Nav2 interface not available!");
      return boost::signals2::connection();
    }
  }

  template <typename T>
  boost::signals2::connection onFeedback(void (T::*callback)(const Feedback &), T * object)
  {
    auto navInterface = getNavInterface();
    if (navInterface)
    {
      return navInterface->onNavigationFeedback(callback, object);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[ClNav2Z] Cannot connect signal - Nav2 interface not available!");
      return boost::signals2::connection();
    }
  }

  // Access methods for components (for advanced usage)
  smacc2::client_core_components::CpActionClient<ActionType> * getActionClient()
  {
    return this->getComponent<smacc2::client_core_components::CpActionClient<ActionType>>();
  }

  components::CpNav2ActionInterface * getNavInterface()
  {
    return this->getComponent<components::CpNav2ActionInterface>();
  }

  // Legacy support - expose signals from nav interface
  SmaccNavigateResultSignal * getSucceededSignal()
  {
    auto navInterface = getNavInterface();
    return navInterface ? &(navInterface->onNavigationSucceeded_) : nullptr;
  }

  SmaccNavigateResultSignal * getAbortedSignal()
  {
    auto navInterface = getNavInterface();
    return navInterface ? &(navInterface->onNavigationAborted_) : nullptr;
  }

  SmaccNavigateResultSignal * getCancelledSignal()
  {
    auto navInterface = getNavInterface();
    return navInterface ? &(navInterface->onNavigationCancelled_) : nullptr;
  }

private:
  std::string actionServerName_;
};

}  // namespace cl_nav2z
