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

#include <cl_px4_mr/client_behaviors/cb_sine_altitude_cruise.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/utils/angle_utils.hpp>

#include <algorithm>

namespace cl_px4_mr
{

CbSineAltitudeCruise::CbSineAltitudeCruise(
  float groundSpeed, float amplitude, float wavelength, float heading)
: groundSpeed_(groundSpeed),
  amplitude_(amplitude),
  wavelength_(std::max(wavelength, 0.1f)),
  heading_(heading)
{
}

void CbSineAltitudeCruise::onEntry()
{
  if (localPosition_ == nullptr || !localPosition_->isValid())
  {
    RCLCPP_ERROR(
      getLogger(), "CbSineAltitudeCruise: no valid local position at entry - posting failure");
    this->postPx4Failure();
    return;
  }

  startX_ = localPosition_->getX();
  startY_ = localPosition_->getY();
  baseZ_ = localPosition_->getZ();

  if (std::isnan(heading_))
  {
    heading_ = localPosition_->getHeading();
  }
  heading_ = wrapPi(heading_);

  // Peak vertical speed of the commanded path; PX4 defaults limit climb/descent
  // to ~2-3 m/s and the flown sine flattens beyond that.
  float peakVerticalSpeed =
    amplitude_ * 2.0f * static_cast<float>(M_PI) * groundSpeed_ / wavelength_;
  if (peakVerticalSpeed > 2.0f)
  {
    RCLCPP_WARN(
      getLogger(),
      "CbSineAltitudeCruise: commanded peak vertical speed %.2f m/s exceeds typical PX4 limits - "
      "the flown path will flatten",
      peakVerticalSpeed);
  }

  arcLength_ = 0.0f;
  lastCmdX_ = startX_;
  lastCmdY_ = startY_;
  lastUpdateTime_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    getLogger(),
    "CbSineAltitudeCruise: cruising heading=%.2f rad v=%.2f m/s A=%.2f m lambda=%.2f m "
    "(continuous - exits only on state change)",
    heading_, groundSpeed_, amplitude_, wavelength_);

  trajectorySetpoint_->setPositionNED(startX_, startY_, baseZ_, heading_);
  active_ = true;
}

void CbSineAltitudeCruise::onExit()
{
  if (active_.exchange(false))
  {
    // Settle at the entry altitude at the last commanded ground position; the
    // offboard keep-alive republishes this until the next state commands otherwise.
    trajectorySetpoint_->setPositionNED(lastCmdX_, lastCmdY_, baseZ_, heading_);
    RCLCPP_INFO(
      getLogger(), "CbSineAltitudeCruise: exiting - settling at base altitude (NED z=%.2f)",
      baseZ_);
  }
}

void CbSineAltitudeCruise::update()
{
  CbPx4ClientBehaviorBase::update();

  if (!active_)
  {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;

  arcLength_ += groundSpeed_ * static_cast<float>(dt);

  float x = startX_ + arcLength_ * std::cos(heading_);
  float y = startY_ + arcLength_ * std::sin(heading_);
  float phase = 2.0f * static_cast<float>(M_PI) * arcLength_ / wavelength_;
  float z = baseZ_ - amplitude_ * std::sin(phase);  // NED: up is negative

  lastCmdX_ = x;
  lastCmdY_ = y;
  trajectorySetpoint_->setPositionNED(x, y, z, heading_);
}

}  // namespace cl_px4_mr
