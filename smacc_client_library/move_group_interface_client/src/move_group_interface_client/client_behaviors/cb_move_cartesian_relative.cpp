/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_cartesian_relative.h>

namespace cl_move_group_interface
{
CbMoveCartesianRelative::CbMoveCartesianRelative()
{
}

CbMoveCartesianRelative::CbMoveCartesianRelative(geometry_msgs::Vector3 offset) : offset_(offset)
{
}

void CbMoveCartesianRelative::onEntry()
{
  this->requiresClient(moveGroupSmaccClient_);

  if (this->group_)
  {
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveRelativeCartesian(&move_group, offset_);
  }
  else
  {
      this->moveRelativeCartesian(&moveGroupSmaccClient_->moveGroupClientInterface, offset_);
  }
}

void CbMoveCartesianRelative::onExit()
{
    
}

// keeps the end efector orientation fixed
void CbMoveCartesianRelative::moveRelativeCartesian(moveit::planning_interface::MoveGroupInterface *movegroupClient,
                                                    geometry_msgs::Vector3 &offset)
{
  std::vector<geometry_msgs::Pose> waypoints;

  // this one was working fine but the issue is that for relative motions it grows up on ABORT-State-Loop pattern. 
  // But, we need current pose because maybe setCurrentPose was not set by previos behaviors. The only solution would be 
  // distinguishing between the executtion error and the planning error with no state change
  //auto referenceStartPose = movegroupClient->getPoseTarget(); 
  auto referenceStartPose = movegroupClient->getCurrentPose();
  movegroupClient->setPoseReferenceFrame(referenceStartPose.header.frame_id);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] RELATIVE MOTION, SOURCE POSE: " << referenceStartPose);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] Offset: " << offset);

  waypoints.push_back(referenceStartPose.pose);  // up and out

  auto endPose = referenceStartPose.pose;

  endPose.position.x += offset.x;
  endPose.position.y += offset.y;
  endPose.position.z += offset.z;

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] DESTINY POSE: " << endPose);

  // target_pose2.position.x -= 0.15;
  waypoints.push_back(endPose);  // left

  movegroupClient->setPoseTarget(endPose);

  float scalinf = 0.5;
  if (scalingFactor_)
    scalinf = *scalingFactor_;

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] Using scaling factor: " << scalinf);
  movegroupClient->setMaxVelocityScalingFactor(scalinf);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = movegroupClient->computeCartesianPath(waypoints,
                                                          0.01,  // eef_step
                                                          0.00,  // jump_threshold
                                                          trajectory);

  moveit::planning_interface::MoveItErrorCode behaviorResult;
  if (fraction != 1.0 || fraction == -1)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] Cartesian plan joint-continuity percentaje. Execution skipped because not 100% of cartesian motion: " << fraction*100<<"%");
    behaviorResult = moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED;
  }
  else
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[CbMoveCartesianRelative] Cartesian plan joint-continuity percentaje: " << fraction);

    moveit::planning_interface::MoveGroupInterface::Plan grasp_pose_plan;

    // grasp_pose_plan.start_state_ = *(moveGroupInterface.getCurrentState());
    grasp_pose_plan.trajectory_ = trajectory;
    behaviorResult = movegroupClient->execute(grasp_pose_plan);
  }

  if (behaviorResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_INFO(nh_->get_logger(), "[CbMoveCartesianRelative] relative motion execution succedded: fraction %lf.", fraction);
    moveGroupSmaccClient_->postEventMotionExecutionSucceded();
    this->postSuccessEvent();
  }
  else
  {
    movegroupClient->setPoseTarget(referenceStartPose); // undo changes since we did not executed the motion
    RCLCPP_INFO(nh_->get_logger(), "[CbMoveCartesianRelative] relative motion execution failed: fraction %lf.", fraction);
    moveGroupSmaccClient_->postEventMotionExecutionFailed();
    this->postFailureEvent();
  }  
}
}  // namespace cl_move_group_interface