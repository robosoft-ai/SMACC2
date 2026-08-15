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

#include <cl_nav2z/client_behaviors/cb_undo_path_backwards.hpp>
#include <cl_nav2z/common.hpp>
#include <cl_nav2z/components/goal_checker_switcher/cp_goal_checker_switcher.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>

namespace cl_nav2z
{
using ::cl_nav2z::odom_tracker::CpOdomTracker;
using ::cl_nav2z::odom_tracker::WorkingMode;

using namespace std::chrono_literals;
using namespace smacc2;

CbUndoPathBackwards::CbUndoPathBackwards(std::optional<CbUndoPathBackwardsOptions> options)
{
  options_ = options;
}

void CbUndoPathBackwards::onEntry()
{
  requiresComponent(odomTracker, ComponentRequirement::HARD);

  odomTracker->logStateString(false);

  CpPlannerSwitcher * plannerSwitcher;
  requiresComponent(plannerSwitcher, ComponentRequirement::HARD);

  nav_msgs::msg::Path forwardpath = odomTracker->getPath();
  // RCLCPP_INFO_STREAM(getLogger(),"[UndoPathBackward] Current path backwards: " << forwardpath);

  odomTracker->setWorkingMode(WorkingMode::CLEAR_PATH);

  nav2_msgs::action::NavigateToPose::Goal goal;

  CpGoalCheckerSwitcher * goalCheckerSwitcher;
  requiresComponent(goalCheckerSwitcher, ComponentRequirement::HARD);

  if (forwardpath.poses.size() > 0)
  {
    goal.pose = forwardpath.poses.front();
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

    // The remote UndoPathGlobalPlanner receives the recorded trail via the
    // odom_tracker_path topic. Right after a popPath (chained undo) its cache may
    // still hold the previous, fully-consumed trail - a few poses under the robot -
    // which would make the goal checker succeed instantly without moving. The
    // tracker republishes at odom rate (~20-30 Hz), so a short settle guarantees
    // the planner sees the restored trail before the goal is accepted.
    // (This runs in the asynchronous onEntry thread: it does not block the state
    // machine. TODO: replace with an explicit ready handshake from the planner.)
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] activating undo navigation planner");

    if (options_ && options_->goalCheckerId_)
    {
      goalCheckerSwitcher->setGoalCheckerId(*options_->goalCheckerId_);
    }
    else
    {
      goalCheckerSwitcher->setGoalCheckerId("undo_path_backwards_goal_checker");
    }

    this->sendGoal(goal);
  }
}

void CbUndoPathBackwards::onExit()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Exiting: undo navigation ");

  // NOTE: odomTracker was resolved in onEntry. Do NOT call requiresComponent here:
  // this method runs in the asynchronous onExit thread while the state machine
  // thread holds m_mutex_ during state disposal and joins this thread - a
  // requiresComponent lookup (which locks m_mutex_) deadlocks the state machine.
  if (this->navigationResult_ == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), getName() << " - [CbUndoPathBackwards] Exiting: undo navigation successful, "
                                "popping odom tracker path");

    odomTracker->popPath();
    odomTracker->logStateString(false);
  }
  else
  {
    RCLCPP_INFO_STREAM(
      getLogger(), getName() << " - [CbUndoPathBackwards] Exiting: undo navigation abort, avoiding "
                                "popping current path");

    odomTracker->logStateString(false);
    // navigation interrupted or aborted. The path may be not totally undone.
    // We keep the odom tracker in its current state, probably in the middle of the undoing process.
    // Could you try to repeat the behavior?
  }

  // Stop consuming the recorded path once the behavior is over: CLEAR_PATH must not
  // outlive this behavior, otherwise the tracker keeps eating paths recorded by
  // subsequent behaviors.
  odomTracker->setWorkingMode(WorkingMode::IDLE);
}

}  // namespace cl_nav2z
