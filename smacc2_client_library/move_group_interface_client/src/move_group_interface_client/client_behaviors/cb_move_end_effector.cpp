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
#include <future>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/impl/utils.h>
#include <move_group_interface_client/client_behaviors/cb_move_end_effector.hpp>
#include <move_group_interface_client/common.hpp>

using namespace std::chrono_literals;

namespace cl_move_group_interface
{
CbMoveEndEffector::CbMoveEndEffector() {}

CbMoveEndEffector::CbMoveEndEffector(
  geometry_msgs::msg::PoseStamped target_pose, std::string tip_link)
: targetPose(target_pose)
{
  tip_link_ = tip_link;
}

void CbMoveEndEffector::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
    RCLCPP_DEBUG(
      getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
    moveit::planning_interface::MoveGroupInterface move_group(getNode(), *group_);
    this->moveToAbsolutePose(move_group, targetPose);
    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
  else
  {
    RCLCPP_DEBUG(
      getLogger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
    this->moveToAbsolutePose(*(movegroupClient_->moveGroupClientInterface), targetPose);
    RCLCPP_DEBUG(getLogger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
}

bool CbMoveEndEffector::moveToAbsolutePose(
  moveit::planning_interface::MoveGroupInterface & moveGroupInterface,
  geometry_msgs::msg::PoseStamped & targetObjectPose)
{
  // auto & planningSceneInterface = movegroupClient_->planningSceneInterface;
  RCLCPP_DEBUG(getLogger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  rclcpp::sleep_for(500ms);

  moveGroupInterface.setPlanningTime(1.0);

  RCLCPP_INFO_STREAM(
    getLogger(), "[CbMoveEndEffector] Target End efector Pose: " << targetObjectPose);

  moveGroupInterface.setPoseTarget(targetObjectPose, tip_link_);
  moveGroupInterface.setPoseReferenceFrame(targetObjectPose.header.frame_id);

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success =
    (moveGroupInterface.plan(computedMotionPlan) ==
     moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(
    getLogger(), "[CbMoveEndEffector] Success Visualizing plan 1 (pose goal) %s",
    success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] motion execution succeeded");
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbMoveEndEffector] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }

  RCLCPP_DEBUG(getLogger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  rclcpp::sleep_for(500ms);

  return success;
}

}  // namespace cl_move_group_interface
