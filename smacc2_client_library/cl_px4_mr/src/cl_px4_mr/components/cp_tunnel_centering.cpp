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

#include <cl_px4_mr/components/cp_tunnel_centering.hpp>
#include <cl_px4_mr/utils/level_frame.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace cl_px4_mr
{

namespace
{
constexpr float kInf = std::numeric_limits<float>::infinity();

// k-th smallest of v (v is modified); +inf when v has fewer than k elements
float kthSmallest(std::vector<float> & v, size_t k)
{
  if (v.size() < k || k == 0)
  {
    return kInf;
  }
  std::nth_element(v.begin(), v.begin() + (k - 1), v.end());
  return v[k - 1];
}

int64_t nowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}
}  // namespace

CpTunnelCentering::CpTunnelCentering(TunnelCenteringParams params) : params_(params) {}

CpTunnelCentering::~CpTunnelCentering() {}

void CpTunnelCentering::onInitialize()
{
  this->requiresComponent(subscriber_);
  if (subscriber_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(),
      "CpTunnelCentering: no CpTopicSubscriber<PointCloud2> on this client - centering inactive");
    return;
  }
  subscriber_->onMessageReceived(&CpTunnelCentering::onCloud, this);
  if (params_.levelFrame)
  {
    attitudeSub_ = this->getNode()->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
      std::bind(&CpTunnelCentering::onAttitude, this, std::placeholders::_1));
  }
  lastLog_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    getLogger(),
    "CpTunnelCentering: subscribed - slab %.1f..%.1f m ahead, floor clearance %.2f m (min %.1f), "
    "ceiling clearance %.1f m, max lateral %.1f m, max vertical %.1f m, wall fade %.1f m",
    params_.lookAheadMinM, params_.lookAheadMaxM, params_.floorClearanceM,
    params_.minFloorClearanceM, params_.ceilingClearanceM, params_.maxLateralM,
    params_.maxVerticalM, params_.wallFadeM);
}

bool CpTunnelCentering::valid() const
{
  const int64_t last = lastCloudNs_.load();
  return last > 0 && (nowNs() - last) < 1000000000LL;
}

void CpTunnelCentering::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(attitudeMutex_);
  attitudeQ_ = {msg->q[0], msg->q[1], msg->q[2], msg->q[3]};
  haveAttitude_ = true;
}

void CpTunnelCentering::onCloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::array<float, 9> r{};
  bool level = false;
  if (params_.levelFrame)
  {
    std::array<float, 4> q;
    {
      std::lock_guard<std::mutex> lock(attitudeMutex_);
      level = haveAttitude_;
      q = attitudeQ_;
    }
    if (level)
    {
      levelRotationFromPx4Quaternion(q, r);
    }
  }

  std::vector<float> leftY, rightY, floorZ, ceilZ;
  leftY.reserve(512);
  rightY.reserve(512);
  floorZ.reserve(512);
  ceilZ.reserve(512);

  const float minRangeSq = params_.minRangeM * params_.minRangeM;
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
      continue;
    }
    if (x * x + y * y + z * z < minRangeSq)
    {
      continue;  // own airframe
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
    if (x < -params_.floorLookBackM || x > params_.lookAheadMaxM)
    {
      continue;
    }
    const bool ahead = x >= params_.lookAheadMinM;
    // walls: the band at flight height, in the slab ahead
    if (
      ahead && std::fabs(z) <= params_.wallBandHalfHeightM &&
      std::fabs(y) <= params_.lateralSearchM)
    {
      if (y > 0.0f)
      {
        leftY.push_back(y);
      }
      else if (y < 0.0f)
      {
        rightY.push_back(-y);
      }
    }
    // floor / ceiling: the column along the heading, EXCLUDING the wall band -
    // rock beside the vehicle at sensor height is a wall (lateral channel), not
    // a floor 0.1 m below it (run 22: an arch wall read as "floor 0.1" forced a
    // 1 m climb into the ceiling)
    if (
      std::fabs(y) <= params_.columnHalfWidthM && std::fabs(z) > params_.wallBandHalfHeightM &&
      std::fabs(z) <= params_.verticalSearchM)
    {
      if (z < 0.0f)
      {
        floorZ.push_back(-z);
      }
      else
      {
        ceilZ.push_back(z);
      }
    }
  }

  // nearest surfaces, robust to a few stray returns
  const size_t kw = static_cast<size_t>(std::max(params_.minWallPoints, 1));
  const size_t kf = static_cast<size_t>(std::max(params_.minFloorPoints, 1));
  const float left = kthSmallest(leftY, kw);
  const float right = kthSmallest(rightY, kw);
  const float floorD = kthSmallest(floorZ, kf);
  const float ceilD = kthSmallest(ceilZ, kf);

  // lateral: toward the midpoint between the walls; needs both, fades when far
  float lateral = 0.0f;
  if (std::isfinite(left) && std::isfinite(right))
  {
    lateral = 0.5f * (left - right);  // > 0: more room on the left -> move left
    const float nearer = std::min(left, right);
    if (nearer > params_.wallFadeM)
    {
      lateral = 0.0f;
    }
    else if (nearer > 0.5f * params_.wallFadeM)
    {
      lateral *= (params_.wallFadeM - nearer) / (0.5f * params_.wallFadeM);
    }
  }
  lateral = std::clamp(lateral, -params_.maxLateralM, params_.maxLateralM);

  // vertical: hold the floor clearance, but stay below the ceiling
  float vertical = 0.0f;
  if (std::isfinite(floorD))
  {
    vertical = params_.floorClearanceM - floorD;  // > 0: climb
  }
  if (std::isfinite(ceilD))
  {
    vertical = std::min(vertical, ceilD - params_.ceilingClearanceM);
  }
  if (std::isfinite(floorD))
  {
    // never below the minimum floor clearance, even when the overhead reading
    // (possibly rising terrain ahead) asks for it
    vertical = std::max(vertical, params_.minFloorClearanceM - floorD);
  }
  vertical = std::clamp(vertical, -params_.maxVerticalM, params_.maxVerticalM);

  const float a = std::clamp(params_.smoothing, 0.0f, 0.95f);
  const bool first = lastCloudNs_.load() == 0;
  lateral_ = first ? lateral : a * lateral_.load() + (1.0f - a) * lateral;
  vertical_ = first ? vertical : a * vertical_.load() + (1.0f - a) * vertical;
  left_ = left;
  right_ = right;
  floor_ = floorD;
  ceiling_ = ceilD;
  lastCloudNs_ = nowNs();

  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - lastLog_).count() >= 2.0)
  {
    lastLog_ = now;
    RCLCPP_INFO(
      getLogger(),
      "CpTunnelCentering: walls L %.1f R %.1f, floor %.1f, ceiling %.1f m -> lateral %+.2f, "
      "vertical %+.2f m",
      left, right, floorD, ceilD, lateral_.load(), vertical_.load());
  }
}

}  // namespace cl_px4_mr
