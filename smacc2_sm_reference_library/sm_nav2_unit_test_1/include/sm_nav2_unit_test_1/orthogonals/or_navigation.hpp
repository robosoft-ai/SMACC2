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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/amcl/cp_amcl.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_nav2_unit_test_1
{

class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_nav2z::ClNav2Z>();

    // Create required components for navigation behaviors
    // CpPose: provides robot pose from TF
    client->createComponent<cl_nav2z::CpPose>("base_link", "map");

    // CpOdomTracker: tracks odometry path for undo operations
    client->createComponent<cl_nav2z::odom_tracker::CpOdomTracker>();

    // CpPlannerSwitcher: switches between planners/controllers
    client->createComponent<cl_nav2z::CpPlannerSwitcher>();

    // CpGoalCheckerSwitcher: switches goal checker algorithms
    client->createComponent<cl_nav2z::CpGoalCheckerSwitcher>();

    // CpAmcl: provides initial pose setting for AMCL localization
    client->createComponent<cl_nav2z::CpAmcl>();
  }
};

}  // namespace sm_nav2_unit_test_1
