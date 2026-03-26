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

#include <cl_px4_mr/client_behaviors/cb_follow_waypoints.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CbFollowWaypoints::CbFollowWaypoints(
  std::vector<std::array<float, 4>> waypoints, float xyTol, float zTol)
: waypoints_(std::move(waypoints)), xyTol_(xyTol), zTol_(zTol)
{
}

void CbFollowWaypoints::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(goalChecker_);

  this->getStateMachine()->createSignalConnection(
    goalChecker_->onGoalReached_, &CbFollowWaypoints::onGoalReachedCallback, this);

  if (waypoints_.empty())
  {
    RCLCPP_WARN(getLogger(), "CbFollowWaypoints: no waypoints provided - posting success");
    this->postSuccessEvent();
    return;
  }

  RCLCPP_INFO(getLogger(), "CbFollowWaypoints: following %zu waypoints", waypoints_.size());

  currentIndex_ = 0;
  commandCurrentWaypoint();
}

void CbFollowWaypoints::onExit() { goalChecker_->clearGoal(); }

void CbFollowWaypoints::commandCurrentWaypoint()
{
  const auto & wp = waypoints_[currentIndex_];
  float x = wp[0];
  float y = wp[1];
  float z = wp[2];
  float yaw = wp[3];

  RCLCPP_INFO(
    getLogger(), "CbFollowWaypoints: commanding waypoint %zu/%zu [%.2f, %.2f, %.2f] yaw=%.2f",
    currentIndex_ + 1, waypoints_.size(), x, y, z, yaw);

  trajectorySetpoint_->setPositionNED(x, y, z, yaw);
  goalChecker_->setGoal(x, y, z, xyTol_, zTol_);
}

void CbFollowWaypoints::onGoalReachedCallback()
{
  currentIndex_++;

  if (currentIndex_ >= waypoints_.size())
  {
    RCLCPP_INFO(
      getLogger(), "CbFollowWaypoints: all %zu waypoints reached - posting success",
      waypoints_.size());
    this->postSuccessEvent();
  }
  else
  {
    commandCurrentWaypoint();
  }
}

}  // namespace cl_px4_mr
