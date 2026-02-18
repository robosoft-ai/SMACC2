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

#include <cl_px4_mr/client_behaviors/cb_orbit_location.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CbOrbitLocation::CbOrbitLocation(
  float centerX, float centerY, float altitude, float radius, float angularVelocity, int numOrbits)
: centerX_(centerX),
  centerY_(centerY),
  altitude_(altitude),
  radius_(radius),
  angularVelocity_(angularVelocity),
  numOrbits_(numOrbits)
{
}

void CbOrbitLocation::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(localPosition_);

  // Compute starting angle from current position relative to center
  float dx = localPosition_->getX() - centerX_;
  float dy = localPosition_->getY() - centerY_;
  startAngle_ = std::atan2(dy, dx);
  currentAngle_ = startAngle_;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  // Command initial orbit position
  float x = centerX_ + radius_ * std::cos(currentAngle_);
  float y = centerY_ + radius_ * std::sin(currentAngle_);
  float z = -altitude_;              // NED: up is negative
  float yaw = currentAngle_ + M_PI;  // Face toward center

  RCLCPP_INFO(
    getLogger(), "CbOrbitLocation: starting orbit center=[%.2f, %.2f] r=%.2f alt=%.2f orbits=%d",
    centerX_, centerY_, radius_, altitude_, numOrbits_);

  trajectorySetpoint_->setPositionNED(x, y, z, yaw);
}

void CbOrbitLocation::onExit() {}

void CbOrbitLocation::update()
{
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;

  // Advance angle
  currentAngle_ += angularVelocity_ * dt;

  // Compute new position on circle
  float x = centerX_ + radius_ * std::cos(currentAngle_);
  float y = centerY_ + radius_ * std::sin(currentAngle_);
  float z = -altitude_;              // NED
  float yaw = currentAngle_ + M_PI;  // Face toward center

  trajectorySetpoint_->setPositionNED(x, y, z, yaw);

  // Check if we've completed the required number of orbits
  float totalAngle = currentAngle_ - startAngle_;
  float requiredAngle = numOrbits_ * 2.0f * M_PI;

  if (totalAngle >= requiredAngle)
  {
    RCLCPP_INFO(getLogger(), "CbOrbitLocation: %d orbits completed - posting success", numOrbits_);
    this->postSuccessEvent();
  }
}

}  // namespace cl_px4_mr
