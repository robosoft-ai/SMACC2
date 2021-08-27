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

#include <move_base_z_client_plugin/client_behaviors/cb_undo_path_backwards.hpp>
#include <move_base_z_client_plugin/common.hpp>
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.hpp>
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.hpp>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

void CbUndoPathBackwards::onEntry()
{
  listener = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
  auto * odomTracker = moveBaseClient_->getComponent<OdomTracker>();

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();

  nav_msgs::msg::Path forwardpath = odomTracker->getPath();
  // RCLCPP_INFO_STREAM(getLogger(),"[UndoPathBackward] Current path backwards: " << forwardpath);

  odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH);

  ClMoveBaseZ::Goal goal;

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("undo_path_backwards_goal_checker");

  // this line is used to flush/reset backward planner in the case it were already there
  // plannerSwitcher->setDefaultPlanners();
  if (forwardpath.poses.size() > 0)
  {
    goal.pose = forwardpath.poses.front();
    goal.pose.header.stamp = getNode()->now();
    plannerSwitcher->setUndoPathBackwardPlanner();
    moveBaseClient_->sendGoal(goal);
  }
}

void CbUndoPathBackwards::onExit()
{
  auto * odomTracker = moveBaseClient_->getComponent<OdomTracker>();
  odomTracker->popPath();
}

}  // namespace cl_move_base_z
