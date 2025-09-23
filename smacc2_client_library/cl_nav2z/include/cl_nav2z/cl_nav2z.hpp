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

#include <cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc.hpp>

namespace cl_nav2z
{

// Pure component-based ClNav2Z - No legacy support
// Follows cl_keyboard pattern: orchestrator only, behaviors use requiresComponent()
class ClNav2Z : public smacc2::ISmaccClient
{
public:
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
      smacc2::client_core_components::CpActionClient<nav2_msgs::action::NavigateToPose>,
      TOrthogonal, ClNav2Z>(actionServerName_);

    // Create nav2-specific interface component (requires actionClient)
    this->createComponent<components::CpNav2ActionInterface, TOrthogonal, ClNav2Z>();

    RCLCPP_INFO_STREAM(
      this->getLogger(), "[ClNav2Z] Components created for action server: " << actionServerName_);
  }

private:
  std::string actionServerName_;

  // ✅ PURE ORCHESTRATOR PATTERN - ALL LEGACY METHODS REMOVED:
  //
  // ❌ REMOVED: sendGoal() methods - use CpNav2ActionInterface::sendGoal()
  // ❌ REMOVED: signal connection proxy methods - use CpNav2ActionInterface::onNavigationSucceeded()
  // ❌ REMOVED: signal getter methods - signals owned by components
  // ❌ REMOVED: component accessor methods - use requiresComponent()
  // ❌ REMOVED: convenience wrapper methods - use component APIs directly
  // ❌ REMOVED: type aliases - use component types directly
  //
  // ✅ NEW BEHAVIOR PATTERN:
  //   class CbNavigateForward : public CbNav2ZClientBehaviorBase {
  //     void onEntry() override {
  //       CpNav2ActionInterface* navInterface;
  //       this->requiresComponent(navInterface);
  //       navInterface->onNavigationSucceeded(&CbNavigateForward::onSuccess, this);
  //       Goal goal = createForwardGoal();
  //       navInterface->sendGoal(goal);
  //     }
  //   };
  //
  // ✅ SMACC2 SIGNAL COMPLIANCE: All signal connections use createSignalConnection()
  // ✅ FRAMEWORK CONSISTENCY: Matches cl_keyboard pure component pattern
  // ✅ CLEAN SEPARATION: Client orchestrates, components implement, behaviors consume
};

}  // namespace cl_nav2z
