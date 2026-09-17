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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#include <cl_px4_mr/client_behaviors/cb_px4_path_follower_base.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/utils/angle_utils.hpp>

#include <algorithm>
#include <cmath>

namespace cl_px4_mr
{

CbPx4PathFollowerBase::CbPx4PathFollowerBase(PathFollowerParams params) : followerParams_(params) {}

void CbPx4PathFollowerBase::onEntry()
{
  if (localPosition_ == nullptr || trajectorySetpoint_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(), "%s: local position / trajectory setpoint component missing - posting failure",
      behaviorName());
    this->postPx4Failure();
    return;
  }
  if (!localPosition_->isValid())
  {
    RCLCPP_ERROR(
      getLogger(), "%s: no valid local position at entry - posting failure", behaviorName());
    this->postPx4Failure();
    return;
  }

  NedPoint current;
  current.x = localPosition_->getX();
  current.y = localPosition_->getY();
  current.z = localPosition_->getZ();
  current.yaw = localPosition_->getHeading();
  entryHeading_ = current.yaw;

  std::vector<NedPoint> raw = buildPath(current);
  if (raw.empty())
  {
    RCLCPP_ERROR(
      getLogger(), "%s: buildPath() returned no vertices - posting failure", behaviorName());
    this->postPx4Failure();
    return;
  }

  path_.clear();
  if (followerParams_.prependCurrentPosition)
  {
    NedPoint start = current;
    start.yaw = std::numeric_limits<float>::quiet_NaN();
    path_.push_back(start);
  }
  for (const NedPoint & v : raw)
  {
    if (!path_.empty() && nedDistance(path_.back(), v) < followerParams_.minSegmentLength)
    {
      // keep the later vertex's yaw, drop the degenerate segment
      path_.back().yaw = std::isnan(v.yaw) ? path_.back().yaw : v.yaw;
      continue;
    }
    path_.push_back(v);
  }

  cumLen_ = cumulativeLengths(path_);
  totalLen_ = cumLen_.empty() ? 0.0f : cumLen_.back();
  sCarrot_ = 0.0f;
  lastProgressDecile_ = -1;

  const float speed = std::max(followerParams_.groundSpeed, 0.05f);
  const float lag = speed / 0.95f;
  if (followerParams_.leash < 1.2f * lag)
  {
    RCLCPP_WARN(
      getLogger(),
      "%s: leash %.1f m is below 1.2x the expected tracking lag (%.1f m at %.1f m/s) - effective "
      "speed will be throttled to ~%.1f m/s",
      behaviorName(), followerParams_.leash, lag, speed, 0.95f * followerParams_.leash);
  }

  if (!this->hasTimeout() && followerParams_.autoTimeoutFactor > 0.0f)
  {
    const float seconds = std::max(30.0f, followerParams_.autoTimeoutFactor * totalLen_ / speed);
    this->setTimeout(std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000.0f)));
    RCLCPP_INFO(getLogger(), "%s: auto-timeout armed at %.0f s", behaviorName(), seconds);
  }

  RCLCPP_INFO(
    getLogger(),
    "%s: following %zu vertices, %.1f m at %.1f m/s (leash %.1f m) from NED (%.1f, %.1f, %.1f) "
    "to (%.1f, %.1f, %.1f)",
    behaviorName(), path_.size(), totalLen_, speed, followerParams_.leash, path_.front().x,
    path_.front().y, path_.front().z, path_.back().x, path_.back().y, path_.back().z);
  onPathStarted(path_);

  lastCmd_ = commandFor(0.0f);
  trajectorySetpoint_->setPositionNED(lastCmd_.x, lastCmd_.y, lastCmd_.z, lastCmd_.yaw);
  lastUpdateTime_ = std::chrono::steady_clock::now();
  active_ = true;
}

void CbPx4PathFollowerBase::onExit()
{
  if (active_.exchange(false))
  {
    // hold at the last carrot (not hold(): that would step the setpoint back
    // to the lagging vehicle position)
    trajectorySetpoint_->setPositionNED(lastCmd_.x, lastCmd_.y, lastCmd_.z, lastCmd_.yaw);
    RCLCPP_INFO(
      getLogger(), "%s: exiting at %.0f%% - holding at last setpoint (%.1f, %.1f, %.1f)",
      behaviorName(), 100.0f * progressFraction(), lastCmd_.x, lastCmd_.y, lastCmd_.z);
  }
}

void CbPx4PathFollowerBase::update()
{
  CbPx4ClientBehaviorBase::update();

  if (!active_)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  float dt = std::chrono::duration<float>(now - lastUpdateTime_).count();
  lastUpdateTime_ = now;
  dt = std::clamp(dt, 0.0f, 0.5f);

  if (!localPosition_->isValid())
  {
    // freeze the carrot; the watchdog covers a lost position
    return;
  }

  NedPoint vehicle;
  vehicle.x = localPosition_->getX();
  vehicle.y = localPosition_->getY();
  vehicle.z = localPosition_->getZ();

  // advance the carrot, throttled by the leash
  const float sNext = std::min(sCarrot_ + followerParams_.groundSpeed * dt, totalLen_);
  const NedPoint candidate = sampleAtArcLength(path_, cumLen_, sNext);
  if (nedDistance(candidate, vehicle) <= followerParams_.leash)
  {
    sCarrot_ = sNext;
  }

  lastCmd_ = commandFor(sCarrot_);
  trajectorySetpoint_->setPositionNED(lastCmd_.x, lastCmd_.y, lastCmd_.z, lastCmd_.yaw);

  const int decile = static_cast<int>(progressFraction() * 10.0f);
  if (decile != lastProgressDecile_ && decile > 0 && decile < 10)
  {
    lastProgressDecile_ = decile;
    RCLCPP_INFO(
      getLogger(), "%s: %d%% (%.0f / %.0f m)", behaviorName(), decile * 10, sCarrot_, totalLen_);
  }

  // completion: carrot at the end and vehicle within tolerance of the last vertex
  if (sCarrot_ >= totalLen_)
  {
    const NedPoint & end = path_.back();
    const float dxy = nedDistanceXY(vehicle, end);
    const float dz = std::fabs(vehicle.z - end.z);
    if (dxy <= followerParams_.arrivalXyTol && dz <= followerParams_.arrivalZTol)
    {
      active_ = false;
      RCLCPP_INFO(
        getLogger(), "%s: path complete (xy err %.2f m, z err %.2f m) - posting success",
        behaviorName(), dxy, dz);
      onPathCompleted();
      this->postPx4Success();
    }
  }
}

float CbPx4PathFollowerBase::tangentYawAt(size_t segmentIndex) const
{
  if (path_.size() < 2)
  {
    return entryHeading_;
  }
  const size_t i0 = std::min(segmentIndex, path_.size() - 2);
  const NedPoint & a = path_[i0];
  const NedPoint & b = path_[i0 + 1];
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  if (std::hypot(dx, dy) < 1e-3f)
  {
    return entryHeading_;  // vertical segment: keep heading
  }
  return std::atan2(dy, dx);
}

NedPoint CbPx4PathFollowerBase::commandFor(float s)
{
  size_t segment = 0;
  NedPoint cmd = sampleAtArcLength(path_, cumLen_, s, &segment);

  switch (followerParams_.yawMode)
  {
    case YawMode::FIXED:
      cmd.yaw = std::isnan(followerParams_.fixedYaw) ? entryHeading_ : followerParams_.fixedYaw;
      break;
    case YawMode::HOLD_ENTRY:
      cmd.yaw = entryHeading_;
      break;
    case YawMode::PER_VERTEX:
      if (std::isnan(cmd.yaw))
      {
        cmd.yaw = tangentYawAt(segment);
      }
      break;
    case YawMode::TANGENT:
    default:
      cmd.yaw = tangentYawAt(segment);
      break;
  }
  cmd.yaw = wrapPi(cmd.yaw);
  return cmd;
}

}  // namespace cl_px4_mr
