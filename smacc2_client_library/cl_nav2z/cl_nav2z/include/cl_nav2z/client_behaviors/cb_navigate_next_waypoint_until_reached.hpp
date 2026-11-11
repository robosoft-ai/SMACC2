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

#include <cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp>
#include "cb_navigate_next_waypoint.hpp"

namespace cl_nav2z
{
template <typename AsyncCB, typename Orthogonal>
struct EvGoalWaypointReached : sc::event<EvGoalWaypointReached<AsyncCB, Orthogonal>>
{
};

class CbNavigateNextWaypointUntilReached : public CbNavigateNextWaypoint
{
public:
  CbNavigateNextWaypointUntilReached(
    std::string goalWaypointName,
    const std::optional<NavigateNextWaypointOptions> & options = std::nullopt);

  virtual ~CbNavigateNextWaypointUntilReached();

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    CbNavigateNextWaypoint::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postEvGoalWaypointReached_ = [this]()
    { this->postEvent<EvGoalWaypointReached<TSourceObject, TOrthogonal>>(); };
  }

  void onEntry() override;

  void onExit() override;

  void onNavigationActionSuccess(
    const components::CpNav2ActionInterface::WrappedResult & r) override
  {
    navigationResult_ = r.code;

    RCLCPP_INFO(
      getLogger(), "[%s] Propagating success event from action server", getName().c_str());

    this->requiresComponent(waypointsNavigator_, ComponentRequirement::HARD);

    // Empty past the last waypoint or for an unnamed one; never dereference it unchecked.
    auto current_waypoint_name = waypointsNavigator_->getCurrentWaypointName();
    const char * current = current_waypoint_name ? current_waypoint_name->c_str() : "(none)";

    if (current_waypoint_name && *current_waypoint_name == this->goalWaypointName_)
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbNavigateNextWaypointUntilReached] GoalReached current iteration waypoint i: %ld with "
        "name '%s'",
        waypointsNavigator_->getCurrentWaypointIndex(), current);

      this->postEvGoalWaypointReached_();
    }
    else
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbNavigateNextWaypointUntilReached] goal:'%s' current:'%s'. keep navigating.",
        goalWaypointName_.c_str(), current);
    }

    this->postSuccessEvent();
  }

private:
  std::string goalWaypointName_;

  std::function<void()> postEvGoalWaypointReached_;
};
}  // namespace cl_nav2z
