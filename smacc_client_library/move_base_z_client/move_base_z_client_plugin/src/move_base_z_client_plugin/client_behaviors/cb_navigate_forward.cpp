/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_base_z_client_plugin/client_behaviors/cb_navigate_forward.h>
#include <move_base_z_client_plugin/common.h>

#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>
#include <tf2/utils.h>

namespace cl_move_base_z
{
using namespace ::cl_move_base_z::odom_tracker;

CbNavigateForward::CbNavigateForward(float forwardDistance)
{
  this->forwardDistance = forwardDistance;
}

CbNavigateForward::CbNavigateForward() {}

CbNavigateForward::~CbNavigateForward() {}

void CbNavigateForward::onEntry()
{
  // straight motion distance
  double dist = 3.5;

  if (!forwardDistance)
  {
    // second choice parameter server
    if (!this->getCurrentState()->getParam("straight_motion_distance", dist))
    {
      // last choice default value;
    }
  }
  else
  {
    dist = *forwardDistance;
  }

  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[CbNavigateForward] Straight motion distance: " << dist);

  // get current pose
  auto p = moveBaseClient_->getComponent<cl_move_base_z::Pose>();
  auto referenceFrame = p->getReferenceFrame();
  auto currentPoseMsg = p->toPoseMsg();
  tf2::Transform currentPose;
  tf2::fromMsg(currentPoseMsg, currentPose);

  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[CbNavigateForward] current pose: " << currentPoseMsg);

  // force global orientation if it is requested
  if (this->forceInitialOrientation)
  {
    currentPoseMsg.orientation = *forceInitialOrientation;
    RCLCPP_WARN_STREAM(
      getNode()->get_logger(), "[CbNavigateForward] Forcing initial straight motion orientation: "
                                 << currentPoseMsg.orientation);
  }

  // compute forward goal pose
  tf2::Transform forwardDeltaTransform;
  forwardDeltaTransform.setIdentity();
  forwardDeltaTransform.setOrigin(tf2::Vector3(dist, 0, 0));

  tf2::Transform targetPose = currentPose * forwardDeltaTransform;

  ClMoveBaseZ::Goal goal;
  goal.pose.header.frame_id = referenceFrame;
  goal.pose.header.stamp = getNode()->now();
  tf2::toMsg(targetPose, goal.pose.pose);

  RCLCPP_INFO_STREAM(getNode()->get_logger(), "TARGET POSE FORWARD: " << goal.pose.pose);

  geometry_msgs::msg::PoseStamped currentStampedPoseMsg;
  currentStampedPoseMsg.header.frame_id = referenceFrame;
  currentStampedPoseMsg.header.stamp = getNode()->now();
  tf2::toMsg(currentPose, currentStampedPoseMsg.pose);

  odomTracker_ = moveBaseClient_->getComponent<OdomTracker>();
  odomTracker_->pushPath();

  odomTracker_->setStartPoint(currentStampedPoseMsg);
  odomTracker_->setWorkingMode(WorkingMode::RECORD_PATH);

  auto plannerSwitcher = moveBaseClient_->getComponent<PlannerSwitcher>();
  plannerSwitcher->setForwardPlanner();

  auto goalCheckerSwitcher = moveBaseClient_->getComponent<GoalCheckerSwitcher>();
  goalCheckerSwitcher->setGoalCheckerId("forward_goal_checker");

  moveBaseClient_->sendGoal(goal);
}

void CbNavigateForward::onExit() { this->odomTracker_->setWorkingMode(WorkingMode::IDLE); }

}  // namespace cl_move_base_z