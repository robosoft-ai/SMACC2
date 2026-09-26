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

#include <cl_px4_mr/client_behaviors/cb_spiral_up.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CbSpiralUp::CbSpiralUp(SpiralUpParams params) : params_(params) {}

void CbSpiralUp::onEntry()
{
  const float dx = localPosition_->getX() - params_.centerX;
  const float dy = localPosition_->getY() - params_.centerY;
  startAngle_ = std::atan2(dy, dx);
  currentAngle_ = startAngle_;
  startAltitude_ =
    std::isfinite(params_.startAltitudeM) ? params_.startAltitudeM : -localPosition_->getZ();
  lastReportedOrbit_ = -1;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    getLogger(),
    "CbSpiralUp: helix about NED (%.1f, %.1f), radius %.1f m, from %.1f m up %.1f m at %.1f m per "
    "orbit (%.1f orbits), %.2f rad/s",
    static_cast<double>(params_.centerX), static_cast<double>(params_.centerY),
    static_cast<double>(params_.radiusM), static_cast<double>(startAltitude_),
    static_cast<double>(params_.climbTotalM), static_cast<double>(params_.climbPerOrbitM),
    static_cast<double>(params_.climbTotalM / params_.climbPerOrbitM),
    static_cast<double>(params_.angularVelocityRadS));
  command();
}

void CbSpiralUp::command()
{
  const float turned = currentAngle_ - startAngle_;
  const float orbits = std::fabs(turned) / (2.0f * static_cast<float>(M_PI));
  const float climb = std::min(orbits * params_.climbPerOrbitM, params_.climbTotalM);
  const float x = params_.centerX + params_.radiusM * std::cos(currentAngle_);
  const float y = params_.centerY + params_.radiusM * std::sin(currentAngle_);
  const float z = -(startAltitude_ + climb);                   // NED: up is negative
  const float yaw = currentAngle_ + static_cast<float>(M_PI);  // nose toward the axis
  trajectorySetpoint_->setPositionNED(x, y, z, yaw);

  const int orbit = static_cast<int>(orbits);
  if (orbit != lastReportedOrbit_)
  {
    lastReportedOrbit_ = orbit;
    RCLCPP_INFO(
      getLogger(), "CbSpiralUp: orbit %d, commanding %.1f m up", orbit,
      static_cast<double>(startAltitude_ + climb));
  }
}

void CbSpiralUp::update()
{
  CbPx4ClientBehaviorBase::update();

  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;
  currentAngle_ += params_.angularVelocityRadS * static_cast<float>(dt);
  command();

  const float turned = std::fabs(currentAngle_ - startAngle_);
  const float needed =
    params_.climbTotalM / params_.climbPerOrbitM * 2.0f * static_cast<float>(M_PI);
  if (turned >= needed)
  {
    RCLCPP_INFO(
      getLogger(), "CbSpiralUp: %.1f m gained over %.1f orbits - posting success",
      static_cast<double>(params_.climbTotalM), static_cast<double>(turned / (2.0f * M_PI)));
    this->postPx4Success();
  }
}

}  // namespace cl_px4_mr
