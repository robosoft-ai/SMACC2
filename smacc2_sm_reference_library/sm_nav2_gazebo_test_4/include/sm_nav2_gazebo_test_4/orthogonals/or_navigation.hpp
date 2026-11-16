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

#pragma once

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/amcl/cp_amcl.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <cl_nav2z/components/waypoints_navigator/cp_waypoints_navigator.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_nav2_gazebo_test_4
{
class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_nav2z::ClNav2Z>();
    client->createComponent<cl_nav2z::CpPose>("base_link", "map");
    client->createComponent<cl_nav2z::odom_tracker::CpOdomTracker>();
    client->createComponent<cl_nav2z::CpPlannerSwitcher>();
    client->createComponent<cl_nav2z::CpGoalCheckerSwitcher>();
    client->createComponent<cl_nav2z::CpAmcl>();
    // holds the waypoint list and posts EvWaypointN as each one is reached
    client->createComponent<cl_nav2z::CpWaypointNavigator>();
  }
};
}  // namespace sm_nav2_gazebo_test_4
