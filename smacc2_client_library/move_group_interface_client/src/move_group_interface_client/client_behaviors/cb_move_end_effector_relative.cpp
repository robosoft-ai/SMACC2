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

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <move_group_interface_client/client_behaviors/cb_move_end_effector_relative.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/impl/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <future>

#include <move_group_interface_client/common.hpp>

namespace cl_move_group_interface
{
CbMoveEndEffectorRelative::CbMoveEndEffectorRelative() { transform_.rotation.w = 1; }

CbMoveEndEffectorRelative::CbMoveEndEffectorRelative(geometry_msgs::msg::Transform transform)
: transform_(transform)
{
}

void CbMoveEndEffectorRelative::onEntry()
{
  RCLCPP_INFO_STREAM(
    getLogger(),
    "[CbMoveEndEffectorRelative] Transform end effector pose relative: " << transform_);

  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
    moveit::planning_interface::MoveGroupInterface move_group(
      getNode(), moveit::planning_interface::MoveGroupInterface::Options(*(this->group_)));
    this->moveRelative(move_group, this->transform_);
  }
  else
  {
    this->moveRelative(*movegroupClient_->moveGroupClientInterface, this->transform_);
  }
}

void CbMoveEndEffectorRelative::onExit() {}

void CbMoveEndEffectorRelative::moveRelative(
  moveit::planning_interface::MoveGroupInterface & moveGroupInterface,
  geometry_msgs::msg::Transform & transformOffset)
{
  auto referenceStartPose = moveGroupInterface.getCurrentPose();
  tf2::Quaternion currentOrientation;
  tf2::convert(referenceStartPose.pose.orientation, currentOrientation);
  tf2::Quaternion desiredRelativeRotation;

  tf2::convert(transformOffset.rotation, desiredRelativeRotation);

  auto targetOrientation = desiredRelativeRotation * currentOrientation;

  geometry_msgs::msg::PoseStamped targetObjectPose = referenceStartPose;

  targetObjectPose.pose.orientation = tf2::toMsg(targetOrientation);
  targetObjectPose.pose.position.x += transformOffset.translation.x;
  targetObjectPose.pose.position.y += transformOffset.translation.y;
  targetObjectPose.pose.position.z += transformOffset.translation.z;

  moveGroupInterface.setPlanningTime(1.0);

  RCLCPP_INFO_STREAM(
    getLogger(), "[CbMoveEndEffectorRelative] Target End efector Pose: " << targetObjectPose);

  moveGroupInterface.setPoseTarget(targetObjectPose);
  moveGroupInterface.setPoseReferenceFrame("map");

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success =
    (moveGroupInterface.plan(computedMotionPlan) ==
     moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(getLogger(), "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffectorRelative] motion execution succeeded");
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();
    }
    else
    {
      moveGroupInterface.setPoseTarget(
        referenceStartPose);  // undo to avoid abort-repeat state issues with this relative motion
      RCLCPP_INFO(getLogger(), "[CbMoveEndEffectorRelative] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }
  else
  {
    moveGroupInterface.setPoseTarget(
      referenceStartPose);  //undo since we did not executed the motion
    RCLCPP_INFO(getLogger(), "[CbMoveEndEffectorRelative] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }
}
}  // namespace cl_move_group_interface
