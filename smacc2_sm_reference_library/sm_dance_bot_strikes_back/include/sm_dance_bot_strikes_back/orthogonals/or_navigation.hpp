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

#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc_orthogonal.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/components/slam_toolbox/cp_slam_toolbox.hpp>
#include <nav2z_client/components/waypoints_navigator/waypoints_navigator.hpp>

namespace sm_dance_bot_strikes_back
{
using namespace cl_nav2z;
using namespace std::chrono_literals;

class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto movebaseClient = this->createClient<ClNav2Z>();

    // create pose component
    movebaseClient->createComponent<cl_nav2z::Pose>();

    // create planner switcher
    movebaseClient->createComponent<PlannerSwitcher>();

    // create goal checker switcher
    movebaseClient->createComponent<cl_nav2z::GoalCheckerSwitcher>();

    // create odom tracker
    movebaseClient->createComponent<cl_nav2z::odom_tracker::OdomTracker>();

    // create odom tracker
    movebaseClient->createComponent<cl_nav2z::CpSlamToolbox>();

    // create waypoints navigator component
    auto waypointsNavigator = movebaseClient->createComponent<WaypointNavigator>();
    loadWaypointsFromYaml(waypointsNavigator);

    // change this to skip some points of the yaml file, default = 0
    waypointsNavigator->currentWaypoint_ = 0;
  }

  void loadWaypointsFromYaml(WaypointNavigator * waypointsNavigator)
  {
    // if it is the first time and the waypoints navigator is not configured
    std::string planfilepath;

    RCLCPP_INFO_STREAM(getLogger(), "Reasing parameter file from node: " << getNode()->get_name());
    getNode()->declare_parameter<std::string>("waypoints_plan", "");
    if (getNode()->get_parameter("waypoints_plan", planfilepath))
    {
      std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("sm_dance_bot_strikes_back");
      boost::replace_all(planfilepath, "$(pkg_share)", package_share_directory);
      waypointsNavigator->loadWayPointsFromFile(planfilepath);
      RCLCPP_INFO(getLogger(), "waypoints plan: %s", planfilepath.c_str());
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "waypoints plan file not found: NONE");
    }
  }
};
}  // namespace sm_dance_bot_strikes_back
