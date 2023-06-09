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

#include <nav2z_client/client_behaviors/cb_undo_path_backwards.hpp>
#include <nav2z_client/common.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/planner_switcher/planner_switcher.hpp>

namespace cl_nav2z
{
using ::cl_nav2z::odom_tracker::OdomTracker;
using ::cl_nav2z::odom_tracker::WorkingMode;

using namespace std::chrono_literals;

CbUndoPathBackwards::CbUndoPathBackwards(std::optional<CbUndoPathBackwardsOptions> options)
{
  options_ = options;
}

void CbUndoPathBackwards::onEntry()
{
  listener = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
  odomTracker = nav2zClient_->getComponent<OdomTracker>();

  odomTracker->logStateString(false);

  auto plannerSwitcher = nav2zClient_->getComponent<PlannerSwitcher>();

  nav_msgs::msg::Path forwardpath = odomTracker->getPath();
  // RCLCPP_INFO_STREAM(getLogger(),"[UndoPathBackward] Current path backwards: " << forwardpath);

  odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH);

  ClNav2Z::Goal goal;

  auto goalCheckerSwitcher = nav2zClient_->getComponent<GoalCheckerSwitcher>();

  if (options_ && options_->goalCheckerId_)
  {
    goalCheckerSwitcher->setGoalCheckerId(*options_->goalCheckerId_);
  }
  else
  {
    goalCheckerSwitcher->setGoalCheckerId("undo_path_backwards_goal_checker");
  }

  // WARNING: There might be some race condition with the remote undo global planner were the global path was not
  // received yet
  // TODO: waiting notification from global planner that it is loaded
  rclcpp::sleep_for(1s);

  // this line is used to flush/reset backward planner in the case it were already there
  // plannerSwitcher->setDefaultPlanners();
  if (forwardpath.poses.size() > 0)
  {
    goal.pose = forwardpath.poses.front();
    //goal.pose.header.stamp = getNode()->now();
    goal.pose.header.stamp = rclcpp::Time(0);

    if (options_ && options_->undoControllerName_)
    {
      plannerSwitcher->setUndoPathBackwardPlanner(false);
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << getName() << "] Undoing path with controller: " << *options_->undoControllerName_);
      plannerSwitcher->setDesiredController(*options_->undoControllerName_);
      plannerSwitcher->commitPublish();
    }
    else
    {
      plannerSwitcher->setUndoPathBackwardPlanner();
    }

    this->sendGoal(goal);
  }
}

void CbUndoPathBackwards::onExit()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Exiting: undo navigation ");

  if (this->navigationResult_ == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), getName() << " - [CbUndoPathBackwards] Exiting: undo navigation successful, "
                                "popping odom tracker path");
    odomTracker = nav2zClient_->getComponent<OdomTracker>();
    odomTracker->popPath();

    odomTracker->logStateString(false);
  }
  else
  {
    RCLCPP_INFO_STREAM(
      getLogger(), getName() << " - [CbUndoPathBackwards] Exiting: undo navigation abort, avoiding "
                                "popping current path");

    odomTracker = nav2zClient_->getComponent<OdomTracker>();
    odomTracker->logStateString(false);
    // navigation interrupted or aborted. The path may be not totally undone.
    // We keep the odom tracker in its current state, probably in the middle of the undoing process.
    // Could you try to repeat the behavior?
  }
}

}  // namespace cl_nav2z
