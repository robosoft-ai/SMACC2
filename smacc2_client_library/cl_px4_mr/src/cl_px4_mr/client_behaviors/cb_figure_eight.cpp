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

#include <cl_px4_mr/client_behaviors/cb_figure_eight.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CbFigureEight::CbFigureEight(
  float centerX, float centerY, float altitude, float size, float speed, int numLoops)
: centerX_(centerX),
  centerY_(centerY),
  altitude_(altitude),
  size_(size),
  speed_(speed),
  numLoops_(numLoops)
{
}

void CbFigureEight::onEntry()
{
  this->requiresComponent(trajectorySetpoint_);

  t_ = 0.0f;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  // Command initial position (t=0: x=size, y=0 relative to center)
  float x = centerX_ + size_;
  float y = centerY_;
  float z = -altitude_;  // NED

  RCLCPP_INFO(
    getLogger(), "CbFigureEight: starting figure-8 center=[%.2f, %.2f] alt=%.2f size=%.2f loops=%d",
    centerX_, centerY_, altitude_, size_, numLoops_);

  trajectorySetpoint_->setPositionNED(x, y, z, 0.0f);
}

void CbFigureEight::onExit() {}

void CbFigureEight::update()
{
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;

  t_ += speed_ * dt;

  // Lemniscate of Bernoulli parametric equations
  float sinT = std::sin(t_);
  float cosT = std::cos(t_);
  float denom = 1.0f + sinT * sinT;

  float localX = size_ * cosT / denom;
  float localY = size_ * sinT * cosT / denom;

  float x = centerX_ + localX;
  float y = centerY_ + localY;
  float z = -altitude_;  // NED

  // Compute derivatives for yaw (face direction of travel)
  float sinT2 = sinT * sinT;
  float cosT2 = cosT * cosT;
  float denom2 = denom * denom;
  float dxdt = size_ * (-sinT * (1.0f + sinT2) - cosT * 2.0f * sinT * cosT) / denom2;
  float dydt =
    size_ * ((cosT2 - sinT2) * (1.0f + sinT2) - sinT * cosT * 2.0f * sinT * cosT) / denom2;
  float yaw = std::atan2(dydt, dxdt);

  trajectorySetpoint_->setPositionNED(x, y, z, yaw);

  // Check completion
  float requiredT = numLoops_ * 2.0f * M_PI;
  if (t_ >= requiredT)
  {
    RCLCPP_INFO(getLogger(), "CbFigureEight: %d loops completed - posting success", numLoops_);
    this->postSuccessEvent();
  }
}

}  // namespace cl_px4_mr
