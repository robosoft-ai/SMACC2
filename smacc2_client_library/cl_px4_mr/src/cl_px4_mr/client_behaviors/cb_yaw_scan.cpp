// Copyright 2026 RobosoftAI Inc.
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

#include <cl_px4_mr/client_behaviors/cb_yaw_scan.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/utils/angle_utils.hpp>

#include <algorithm>

namespace cl_px4_mr
{

CbYawScan::CbYawScan(float amplitude, float period, float baseHeading)
: amplitude_(amplitude), period_(std::max(period, 0.1f)), baseHeading_(baseHeading)
{
}

void CbYawScan::onEntry()
{
  if (localPosition_ == nullptr || !localPosition_->isValid())
  {
    RCLCPP_ERROR(getLogger(), "CbYawScan: no valid local position at entry - posting failure");
    this->postPx4Failure();
    return;
  }

  holdX_ = localPosition_->getX();
  holdY_ = localPosition_->getY();
  holdZ_ = localPosition_->getZ();

  if (std::isnan(baseHeading_))
  {
    baseHeading_ = localPosition_->getHeading();
  }
  baseHeading_ = wrapPi(baseHeading_);

  // Peak yaw rate of the commanded sweep; PX4's autonomous yaw rate limit is
  // ~0.8 rad/s by default and the flown sweep clips beyond it.
  float peakYawRate = amplitude_ * 2.0f * static_cast<float>(M_PI) / period_;
  if (peakYawRate > 0.8f)
  {
    RCLCPP_WARN(
      getLogger(),
      "CbYawScan: commanded peak yaw rate %.2f rad/s exceeds typical PX4 limits - the flown "
      "sweep will clip",
      peakYawRate);
  }

  elapsed_ = 0.0f;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    getLogger(),
    "CbYawScan: scanning around heading=%.2f rad A=%.2f rad T=%.2f s "
    "(continuous - exits only on state change)",
    baseHeading_, amplitude_, period_);

  trajectorySetpoint_->setPositionNED(holdX_, holdY_, holdZ_, baseHeading_);
  active_ = true;
}

void CbYawScan::onExit()
{
  if (active_.exchange(false))
  {
    // Restore the base heading; the offboard keep-alive republishes this until
    // the next state commands otherwise.
    trajectorySetpoint_->setPositionNED(holdX_, holdY_, holdZ_, baseHeading_);
    RCLCPP_INFO(getLogger(), "CbYawScan: exiting - restoring base heading %.2f rad", baseHeading_);
  }
}

void CbYawScan::update()
{
  CbPx4ClientBehaviorBase::update();

  if (!active_)
  {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;

  elapsed_ += static_cast<float>(dt);

  float phase = 2.0f * static_cast<float>(M_PI) * elapsed_ / period_;
  float yaw = wrapPi(baseHeading_ + amplitude_ * std::sin(phase));

  trajectorySetpoint_->setPositionNED(holdX_, holdY_, holdZ_, yaw);
}

}  // namespace cl_px4_mr
