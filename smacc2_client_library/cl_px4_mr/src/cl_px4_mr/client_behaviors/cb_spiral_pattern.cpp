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

#include <cl_px4_mr/client_behaviors/cb_spiral_pattern.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CbSpiralPattern::CbSpiralPattern(
  float centerX, float centerY, float altitude, float maxRadius, float spacing, float speed)
: centerX_(centerX),
  centerY_(centerY),
  altitude_(altitude),
  maxRadius_(maxRadius),
  spacing_(spacing),
  speed_(speed)
{
}

void CbSpiralPattern::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);
  this->requiresComponent(localPosition_);

  theta_ = 0.0f;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  // Initial position is the spiral center
  float z = -altitude_;  // NED: up is negative

  RCLCPP_INFO(
    getLogger(),
    "CbSpiralPattern: starting spiral center=[%.2f, %.2f] alt=%.2f maxR=%.2f spacing=%.2f "
    "speed=%.2f",
    centerX_, centerY_, altitude_, maxRadius_, spacing_, speed_);

  trajectorySetpoint_->setPositionNED(centerX_, centerY_, z, 0.0f);
}

void CbSpiralPattern::onExit() {}

void CbSpiralPattern::update()
{
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;

  // Archimedean spiral: r(theta) = a * theta
  // where a = spacing / (2*PI) so each revolution adds 'spacing' meters of radius
  float a = spacing_ / (2.0f * M_PI);

  // Current radius
  float r = a * theta_;

  // Adaptive angular velocity to maintain constant linear speed
  // Arc-length speed: ds/dt = sqrt(r^2 + a^2) * dtheta/dt
  // Solving for dtheta/dt: omega = speed / sqrt(r^2 + a^2)
  float omega = speed_ / std::sqrt(r * r + a * a);

  // Advance angle
  theta_ += omega * dt;

  // Compute new position
  float r_new = a * theta_;
  float x = centerX_ + r_new * std::cos(theta_);
  float y = centerY_ + r_new * std::sin(theta_);
  float z = -altitude_;  // NED

  // Yaw: face direction of travel (derivatives of Archimedean spiral)
  // dx/dtheta = a*cos(theta) - r*sin(theta)
  // dy/dtheta = a*sin(theta) + r*cos(theta)
  float dxdt = a * std::cos(theta_) - r_new * std::sin(theta_);
  float dydt = a * std::sin(theta_) + r_new * std::cos(theta_);
  float yaw = std::atan2(dydt, dxdt);

  trajectorySetpoint_->setPositionNED(x, y, z, yaw);

  // Check completion
  if (r_new >= maxRadius_)
  {
    RCLCPP_INFO(
      getLogger(), "CbSpiralPattern: max radius %.2f reached (r=%.2f) - posting success",
      maxRadius_, r_new);
    this->postSuccessEvent();
  }
}

}  // namespace cl_px4_mr
