/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/client_behaviors/cb_navigate_backwards.h>
#include <move_base_z_client_plugin/common.h>
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <tf2/utils.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

CbNavigateBackwards::CbNavigateBackwards(float backwardDistance)
{
  if (backwardDistance < 0)
  {
    RCLCPP_ERROR(getNode()->get_logger(), "cb backward: distance must be greater or equal than 0");
    this->backwardDistance = 0;
  }
  this->backwardDistance = backwardDistance;
}

CbNavigateBackwards::CbNavigateBackwards() {}

void CbNavigateBackwards::onEntry()
{
  // straight motion distance
  double dist = 3.5;

  if (!backwardDistance)
  {
    this->getCurrentState()->param("straight_motion_distance", dist);
  }
  else
  {
    dist = *backwardDistance;
  }

  this->listener = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[CbNavigateBackwards] Straight backwards motion distance: " << dist);

  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();

  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);
  tf2::Transform forwardDeltaTransform;
  forwardDeltaTransform.setIdentity();
  forwardDeltaTransform.setOrigin(tf2::Vector3(-dist, 0, 0));
  tf2::Transform targetPose = currentPose * forwardDeltaTransform;
  ClMoveBaseZ::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();
  tf2::toMsg(targetPose, goal.pose.pose);
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[CbNavigateBackwards] TARGET POSE BACKWARDS: " << goal.pose);

  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
  if (odomTracker_ != nullptr)
  {
    this->odomTracker_->clearPath();
    this->odomTracker_->setStartPoint(currentPoseMsg);
    this->odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);
  }

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setBackwardPlanner();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("backward_goal_checker");

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateBackwards::onExit()
{
  if (odomTracker_)
  {
    this->odomTracker_->setWorkingMode(WorkingMode::IDLE);
  }
}

}  // namespace cl_move_base_z
