/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_end_effector.h>
// #include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <future>

namespace cl_move_group_interface
{
CbMoveEndEffector::CbMoveEndEffector()
{
}

CbMoveEndEffector::CbMoveEndEffector(geometry_msgs::PoseStamped target_pose, std::string tip_link)
  : targetPose(target_pose)
{
  tip_link_ = tip_link;
}

void CbMoveEndEffector::onEntry()
{
  this->requiresClient(movegroupClient_);

  if (this->group_)
  {
      RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveToAbsolutePose(move_group, targetPose);
      RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
  else
  {
      RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEfector] new thread started to move absolute end effector");
      this->moveToAbsolutePose(movegroupClient_->moveGroupClientInterface, targetPose);
      RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEfector] to move absolute end effector thread destroyed");
  }
}

bool CbMoveEndEffector::moveToAbsolutePose(moveit::planning_interface::MoveGroupInterface &moveGroupInterface,
                                           geometry_msgs::PoseStamped &targetObjectPose)
{
  auto& planningSceneInterface = movegroupClient_->planningSceneInterface;
  RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  ros::Duration(0.5).sleep();

  moveGroupInterface.setPlanningTime(1.0);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveEndEffector] Target End efector Pose: " << targetObjectPose);

  moveGroupInterface.setPoseTarget(targetObjectPose, tip_link_);
  moveGroupInterface.setPoseReferenceFrame(targetObjectPose.header.frame_id);

  moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;
  bool success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO_NAMED(nh_->get_logger(), "CbMoveEndEffector", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success)
  {
    auto executionResult = moveGroupInterface.execute(computedMotionPlan);

    if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(nh_->get_logger(), "[CbMoveEndEffector] motion execution succedded");
      movegroupClient_->postEventMotionExecutionSucceded();
      this->postSuccessEvent();

    }
    else
    {
      RCLCPP_INFO(nh_->get_logger(), "[CbMoveEndEffector] motion execution failed");
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }
  else
  {
    RCLCPP_INFO(nh_->get_logger(), "[CbMoveEndEffector] motion execution failed");
    movegroupClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }

  RCLCPP_DEBUG(nh_->get_logger(), "[CbMoveEndEffector] Synchronous sleep of 1 seconds");
  ros::Duration(0.5).sleep();

  return success;
}

}  // namespace cl_move_group_interface