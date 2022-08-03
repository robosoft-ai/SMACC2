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
  template<typename AsyncCB, typename Orthogonal>
  struct EvGoalWaypointReached: sc::event<EvGoalWaypointReached<AsyncCB, Orthogonal>>
  {
  };
  
class CbNavigateNextWaypointUntilReached : public CbNavigateNextWaypoint
{
public:
  CbNavigateNextWaypointUntilReached(std::string goalWaypointName,
                                 std::optional<NavigateNextWaypointOptions> options = std::nullopt);

  virtual ~CbNavigateNextWaypointUntilReached();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    CbNavigateNextWaypoint::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    postEvGoalWaypointReached_ = [this]() {
      this->postEvent<EvGoalWaypointReached<TSourceObject,TOrthogonal>>();
    };
  }

  void onEntry() override;

  void onExit() override;

private:
  std::string goalWaypointName_;

  std::function<void()> postEvGoalWaypointReached_;
};
}  // namespace cl_nav2z
