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

#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

namespace cl_px4_mr
{

CpVehicleLocalPosition::CpVehicleLocalPosition() {}

CpVehicleLocalPosition::~CpVehicleLocalPosition() {}

void CpVehicleLocalPosition::onInitialize()
{
  auto node = this->getNode();
  subscriber_ = node->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/fmu/out/vehicle_local_position", rclcpp::SensorDataQoS(),
    std::bind(&CpVehicleLocalPosition::onPositionMessage, this, std::placeholders::_1));
  RCLCPP_INFO(getLogger(), "CpVehicleLocalPosition: subscribed to /fmu/out/vehicle_local_position");
}

void CpVehicleLocalPosition::onPositionMessage(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  bool resetDetected = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    x_ = msg->x;
    y_ = msg->y;
    z_ = msg->z;
    heading_ = msg->heading;
    valid_ = msg->xy_valid && msg->z_valid;

    // global reference of the local frame
    xyGlobal_ = msg->xy_global;
    zGlobal_ = msg->z_global;
    refAlt_ = msg->ref_alt;
    if (msg->xy_global && (!projection_.isInitialized() || msg->ref_timestamp != refTimestamp_))
    {
      refLat_ = msg->ref_lat;
      refLon_ = msg->ref_lon;
      refTimestamp_ = msg->ref_timestamp;
      projection_.initReference(refLat_, refLon_, refTimestamp_);
      RCLCPP_INFO(
        getLogger(),
        "CpVehicleLocalPosition: global reference set - NED origin at lat=%.7f lon=%.7f "
        "alt=%.1f m AMSL",
        refLat_, refLon_, static_cast<double>(refAlt_));
    }

    // EKF origin resets
    if (!firstMessage_)
    {
      if (msg->xy_reset_counter != xyResetCounter_)
      {
        RCLCPP_WARN(
          getLogger(),
          "CpVehicleLocalPosition: local XY reset #%u (delta_xy=[%.2f, %.2f]); running paths "
          "are NOT re-projected",
          msg->xy_reset_counter, static_cast<double>(msg->delta_xy[0]),
          static_cast<double>(msg->delta_xy[1]));
        resetDetected = true;
      }
      if (msg->z_reset_counter != zResetCounter_)
      {
        RCLCPP_WARN(
          getLogger(),
          "CpVehicleLocalPosition: local Z reset #%u (delta_z=%.2f); running paths are NOT "
          "re-projected",
          msg->z_reset_counter, static_cast<double>(msg->delta_z));
        resetDetected = true;
      }
    }
    xyResetCounter_ = msg->xy_reset_counter;
    zResetCounter_ = msg->z_reset_counter;
    firstMessage_ = false;
  }

  if (resetDetected)
  {
    onLocalPositionReset_();
  }
  onPositionReceived_();
}

float CpVehicleLocalPosition::getX() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return x_;
}

float CpVehicleLocalPosition::getY() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return y_;
}

float CpVehicleLocalPosition::getZ() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return z_;
}

float CpVehicleLocalPosition::getHeading() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return heading_;
}

bool CpVehicleLocalPosition::isValid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return valid_;
}

bool CpVehicleLocalPosition::globalRefValid() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return xyGlobal_ && projection_.isInitialized();
}

double CpVehicleLocalPosition::getRefLat() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return refLat_;
}

double CpVehicleLocalPosition::getRefLon() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return refLon_;
}

float CpVehicleLocalPosition::getRefAlt() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return refAlt_;
}

uint64_t CpVehicleLocalPosition::getRefTimestamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return refTimestamp_;
}

uint8_t CpVehicleLocalPosition::getXyResetCounter() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return xyResetCounter_;
}

uint8_t CpVehicleLocalPosition::getZResetCounter() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return zResetCounter_;
}

bool CpVehicleLocalPosition::projectToNed(double lat, double lon, float & x, float & y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!xyGlobal_ || !projection_.isInitialized())
  {
    return false;
  }
  projection_.project(lat, lon, x, y);
  return true;
}

bool CpVehicleLocalPosition::reprojectFromNed(float x, float y, double & lat, double & lon) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!xyGlobal_ || !projection_.isInitialized())
  {
    return false;
  }
  projection_.reproject(x, y, lat, lon);
  return true;
}

}  // namespace cl_px4_mr
