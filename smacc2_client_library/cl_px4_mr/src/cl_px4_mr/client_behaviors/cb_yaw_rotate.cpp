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

#include <cl_px4_mr/client_behaviors/cb_yaw_rotate.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CbYawRotate::CbYawRotate(float targetYawRad, bool relative)
: targetYawRad_(targetYawRad), relative_(relative)
{
}

void CbYawRotate::onEntry()
{
  float currentYaw = localPosition_->getHeading();

  if (relative_)
  {
    absoluteTargetYaw_ = currentYaw + targetYawRad_;
  }
  else
  {
    absoluteTargetYaw_ = targetYawRad_;
  }

  // Normalize to [-PI, PI]
  absoluteTargetYaw_ = std::atan2(std::sin(absoluteTargetYaw_), std::cos(absoluteTargetYaw_));

  float curX = localPosition_->getX();
  float curY = localPosition_->getY();
  float curZ = localPosition_->getZ();

  RCLCPP_INFO(
    getLogger(), "CbYawRotate: rotating from %.2f to %.2f rad (relative=%d)", currentYaw,
    absoluteTargetYaw_, relative_);

  trajectorySetpoint_->setPositionNED(curX, curY, curZ, absoluteTargetYaw_);
}

void CbYawRotate::onExit() {}

void CbYawRotate::update()
{
  CbPx4ClientBehaviorBase::update();

  float currentYaw = localPosition_->getHeading();

  // Compute shortest angular distance
  float diff = absoluteTargetYaw_ - currentYaw;
  diff = std::atan2(std::sin(diff), std::cos(diff));

  if (std::abs(diff) < 0.1f)
  {
    RCLCPP_INFO(
      getLogger(), "CbYawRotate: target yaw reached (error=%.3f rad) - posting success",
      std::abs(diff));
    this->postPx4Success();
  }
}

}  // namespace cl_px4_mr
