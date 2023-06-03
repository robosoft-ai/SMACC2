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
    std::optional<NavigateNextWaypointOptions> options = std::nullopt);

  virtual ~CbNavigateNextWaypointUntilReached();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(nav2zClient_);
    CbNavigateNextWaypoint::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postEvGoalWaypointReached_ = [this]()
    {
      this->postEvent<EvGoalWaypointReached<TSourceObject, TOrthogonal>>();

      // nav2zClient_->onSucceeded(&CbNavigateNextWaypointUntilReached::onWaypointReached, this);
    };
  }

  void onEntry() override;

  void onExit() override;

  void onNavigationActionSuccess(const ClNav2Z::WrappedResult & r) override
  {
    // if (!isOwnActionResponse(r))
    // {
    //   RCLCPP_WARN(
    //     getLogger(), "[%s] Propagating success event skipped. Action response is not ours.",
    //     getName().c_str());
    //   return;
    // }

    navigationResult_ = r.code;

    RCLCPP_INFO(
      getLogger(), "[%s] Propagating success event from action server", getName().c_str());

    waypointsNavigator_ = nav2zClient_->getComponent<CpWaypointNavigator>();

    auto current_waypoint_name = waypointsNavigator_->getCurrentWaypointName();

    if (current_waypoint_name == this->goalWaypointName_)
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbNavigateNextWaypointUntilReached] GoalReached current iteration waypoint i: %ld with "
        "name '%s'",
        waypointsNavigator_->getCurrentWaypointIndex(), current_waypoint_name->c_str());

      this->postEvGoalWaypointReached_();
    }
    else
    {
      RCLCPP_INFO(
        getLogger(),
        "[CbNavigateNextWaypointUntilReached] goal:'%s' current:'%s'. keep navigating.",
        goalWaypointName_.c_str(), current_waypoint_name->c_str());
    }

    this->postSuccessEvent();
  }

private:
  std::string goalWaypointName_;

  std::function<void()> postEvGoalWaypointReached_;
};
}  // namespace cl_nav2z
