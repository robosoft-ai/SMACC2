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

// #include <moveit/kinematic_constraints/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <future>

#include <tf2/impl/utils.h>
#include <move_group_interface_client/client_behaviors/cb_move_synchronized_link_goals.hpp>
#include <move_group_interface_client/common.hpp>

using namespace std::chrono_literals;

namespace cl_move_group_interface
{
CbMoveSynchronizedLinkGoals::CbMoveSynchronizedLinkGoals(
  std::vector<geometry_msgs::msg::PoseStamped> targetPoses, std::vector<std::string> tip_links)
: targetPoses_(targetPoses), tip_links_(tip_links)
{
}

void CbMoveSynchronizedLinkGoals::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
    RCLCPP_DEBUG(
      getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
    moveit::planning_interface::MoveGroupInterface move_group(getNode(), *group_);
    this->moveToAbsolutePose(move_group);
    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
  else
  {
    RCLCPP_DEBUG(
      getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
    this->moveToAbsolutePose(*(movegroupClient_->moveGroupClientInterface));
    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
}

bool CbMoveSynchronizedLinkGoals::moveToAbsolutePose(
  moveit::planning_interface::MoveGroupInterface & moveGroupInterface)
{
  // auto & planningSceneInterface = movegroupClient_->planningSceneInterface;
  RCLCPP_DEBUG(getLogger(), "[CbMoveSynchronizedLinkGoals] Synchronous sleep of 1 seconds");
  rclcpp::sleep_for(500ms);

  moveGroupInterface.setPlanningTime(1.0);

  for (int i = 0; i < targetPoses_.size(); i++)
  {
    auto & targetObjectPose = targetPoses_[i];
    auto & tip_link = tip_links_[i];

    moveGroupInterface.setPoseTarget(targetObjectPose, tip_link);
  }

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success =
    (moveGroupInterface.plan(computedMotionPlan) ==
     moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(
    getLogger(), "[CbMoveSynchronizedLinkGoals] Success Visualizing plan 1 (pose goal) %s",
    success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(getLogger(), "[CbMoveSynchronizedLinkGoals] motion execution succeeded");
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbMoveSynchronizedLinkGoals] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbMoveSynchronizedLinkGoals] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }

  RCLCPP_DEBUG(getLogger(), "[CbMoveSynchronizedLinkGoals] Synchronous sleep of 1 seconds");
  rclcpp::sleep_for(500ms);

  return success;
}

}  // namespace cl_move_group_interface
