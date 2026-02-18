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

#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CpGoalChecker::CpGoalChecker() {}

CpGoalChecker::~CpGoalChecker() {}

void CpGoalChecker::onInitialize()
{
  this->requiresComponent(localPosition_);
  RCLCPP_INFO(getLogger(), "CpGoalChecker: initialized");
}

void CpGoalChecker::update()
{
  if (!goalActive_ || !localPosition_ || !localPosition_->isValid()) return;

  float dx = localPosition_->getX() - goalX_;
  float dy = localPosition_->getY() - goalY_;
  float dz = localPosition_->getZ() - goalZ_;
  float xyDist = std::sqrt(dx * dx + dy * dy);
  float zDist = std::abs(dz);

  if (xyDist <= xyTolerance_ && zDist <= zTolerance_)
  {
    RCLCPP_INFO(
      getLogger(), "CpGoalChecker: GOAL REACHED (xy_dist=%.2f z_dist=%.2f)", xyDist, zDist);
    goalActive_ = false;
    onGoalReached_();
  }
}

void CpGoalChecker::setGoal(float x, float y, float z, float xy_tolerance, float z_tolerance)
{
  goalX_ = x;
  goalY_ = y;
  goalZ_ = z;
  xyTolerance_ = xy_tolerance;
  zTolerance_ = z_tolerance;
  goalActive_ = true;
  RCLCPP_INFO(
    getLogger(), "CpGoalChecker: goal set [%.2f, %.2f, %.2f] tol(xy=%.2f z=%.2f)", x, y, z,
    xy_tolerance, z_tolerance);
}

void CpGoalChecker::clearGoal() { goalActive_ = false; }

bool CpGoalChecker::isGoalActive() const { return goalActive_; }

}  // namespace cl_px4_mr
