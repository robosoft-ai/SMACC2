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

#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>

namespace cl_px4_mr
{

CpForwardObstacleGuard::CpForwardObstacleGuard(ForwardObstacleGuardParams params)
: params_(params), cosHalfAngle_(std::cos(params.halfAngleRad))
{
}

CpForwardObstacleGuard::~CpForwardObstacleGuard() {}

void CpForwardObstacleGuard::onInitialize()
{
  this->requiresComponent(subscriber_);
  if (subscriber_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "CpForwardObstacleGuard: no CpTopicSubscriber<PointCloud2> on this client - guard inactive");
    return;
  }
  subscriber_->onMessageReceived(&CpForwardObstacleGuard::onCloud, this);
  if (params_.levelFrame || params_.maxYawRateRadS > 0.0f)
  {
    attitudeSub_ = this->getNode()->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
      std::bind(&CpForwardObstacleGuard::onAttitude, this, std::placeholders::_1));
  }
  RCLCPP_INFO(
    getLogger(),
    "CpForwardObstacleGuard: subscribed - cone %.0f deg, trigger < %.1f m, clear > %.1f m, "
    "%d hits, %d/%d clouds, yaw-rate gate %.2f rad/s, floor cutoff %.1f m, level frame %s",
    params_.halfAngleRad * 180.0 / M_PI, params_.triggerRangeM, params_.clearRangeM,
    params_.minHits, params_.triggerClouds, params_.clearClouds, params_.maxYawRateRadS,
    params_.floorCutoffM, params_.levelFrame ? "on" : "off");
}

void CpForwardObstacleGuard::reset()
{
  const bool was = tooClose_.exchange(false);
  triggerStreak_ = 0;
  clearStreak_ = 0;
  if (was)
  {
    RCLCPP_INFO(
      getLogger(), "CpForwardObstacleGuard: reset (stale flag dropped, last min range %.2f m)",
      lastMinRange_.load());
  }
}

void CpForwardObstacleGuard::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  // yaw rate by finite difference of the NED heading (PX4 does not bridge
  // vehicle_angular_velocity by default); light EMA against quantisation
  const double w = msg->q[0], x = msg->q[1], y = msg->q[2], z = msg->q[3];
  const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  if (lastYawStampUs_ != 0 && msg->timestamp > lastYawStampUs_)
  {
    const double dt = static_cast<double>(msg->timestamp - lastYawStampUs_) * 1e-6;
    double d = yaw - lastYaw_;
    d = std::atan2(std::sin(d), std::cos(d));
    if (dt > 1e-4 && dt < 0.5)
    {
      const float rate = static_cast<float>(d / dt);
      yawRate_ = 0.5f * yawRate_.load() + 0.5f * rate;
    }
  }
  lastYaw_ = yaw;
  lastYawStampUs_ = msg->timestamp;

  std::lock_guard<std::mutex> lock(attitudeMutex_);
  attitudeQ_ = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
  haveAttitude_ = true;
}

bool CpForwardObstacleGuard::levelRotation(std::array<float, 9> & r) const
{
  std::array<float, 4> q;
  {
    std::lock_guard<std::mutex> lock(attitudeMutex_);
    if (!haveAttitude_)
    {
      return false;
    }
    q = attitudeQ_;
  }
  // body FLU -> ENU (px4_ros_com constants), then strip the yaw so the result
  // is the roll/pitch-only rotation into a level, heading-aligned frame
  const tf2::Quaternion kNedEnuQ(0.70710678118654752, 0.70710678118654752, 0.0, 0.0);
  const tf2::Quaternion kFrdFluQ(1.0, 0.0, 0.0, 0.0);
  const tf2::Quaternion qNedFrd(q[1], q[2], q[3], q[0]);
  tf2::Quaternion qEnuFlu = kNedEnuQ * qNedFrd * kFrdFluQ;
  qEnuFlu.normalize();
  const tf2::Matrix3x3 m(qEnuFlu);
  const double yaw = std::atan2(m[1][0], m[0][0]);
  tf2::Matrix3x3 unyaw;
  unyaw.setRPY(0.0, 0.0, -yaw);
  const tf2::Matrix3x3 level = unyaw * m;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      r[i * 3 + j] = static_cast<float>(level[i][j]);
    }
  }
  return true;
}

void CpForwardObstacleGuard::onCloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (params_.maxYawRateRadS > 0.0f && std::fabs(yawRate_.load()) > params_.maxYawRateRadS)
  {
    // turning: the cone is sweeping sideways; keep the current flag, count nothing
    if (++gatedClouds_ == 1)
    {
      RCLCPP_INFO(
        getLogger(), "CpForwardObstacleGuard: yawing at %.2f rad/s - cone gated", yawRate_.load());
    }
    return;
  }
  if (gatedClouds_ > 0)
  {
    RCLCPP_INFO(getLogger(), "CpForwardObstacleGuard: cone active again after %d gated clouds", gatedClouds_);
    gatedClouds_ = 0;
    triggerStreak_ = 0;
  }

  int hits = 0;
  float minRange = std::numeric_limits<float>::infinity();

  std::array<float, 9> r{};
  const bool level = params_.levelFrame && levelRotation(r);

  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz)
  {
    float x = *ix;
    float y = *iy;
    float z = *iz;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      continue;  // gz reports no-return as inf (is_dense = false)
    }
    if (level)
    {
      const float lx = r[0] * x + r[1] * y + r[2] * z;
      const float ly = r[3] * x + r[4] * y + r[5] * z;
      const float lz = r[6] * x + r[7] * y + r[8] * z;
      x = lx;
      y = ly;
      z = lz;
    }
    if (params_.floorCutoffM > 0.0f && z < -params_.floorCutoffM)
    {
      continue;  // the floor, not something in the way
    }
    const float r = std::sqrt(x * x + y * y + z * z);
    if (r < params_.minRangeM || r > params_.clearRangeM)
    {
      continue;
    }
    if (x / r < cosHalfAngle_)
    {
      continue;  // outside the forward cone
    }
    if (r < minRange)
    {
      minRange = r;
    }
    if (r < params_.triggerRangeM)
    {
      ++hits;
    }
  }

  lastMinRange_ = minRange;
  lastHits_ = hits;

  const bool triggering = hits >= params_.minHits;
  // cleared only when nothing in the cone is below the clear range at all
  const bool clear = !(minRange < params_.clearRangeM);

  if (!tooClose_)
  {
    triggerStreak_ = triggering ? triggerStreak_ + 1 : 0;
    if (triggerStreak_ >= params_.triggerClouds)
    {
      tooClose_ = true;
      clearStreak_ = 0;
      RCLCPP_WARN(
        getLogger(), "CpForwardObstacleGuard: OBSTACLE min range %.2f m (%d hits in cone)", minRange,
        hits);
      onObstacleTooClose_();
    }
  }
  else
  {
    clearStreak_ = clear ? clearStreak_ + 1 : 0;
    if (clearStreak_ >= params_.clearClouds)
    {
      tooClose_ = false;
      triggerStreak_ = 0;
      RCLCPP_INFO(getLogger(), "CpForwardObstacleGuard: cleared (min range %.2f m)", minRange);
      onObstacleCleared_();
    }
  }
}

}  // namespace cl_px4_mr
