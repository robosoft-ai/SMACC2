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

#pragma once

#include <cstdint>
#include <mutex>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <cl_px4_mr/utils/geo_utils.hpp>

namespace cl_px4_mr
{

// Caches the fused local NED position from /fmu/out/vehicle_local_position and
// the global reference of that frame (ref_lat/ref_lon/ref_alt = the WGS84
// position of the NED origin). Owns a MapProjection about that reference so
// callers can convert mission lat/lon into the FMU's own local frame.
//
// EKF origin resets (xy_reset_counter / z_reset_counter) are logged and
// signalled but NOT compensated: paths computed before a reset are stale by
// delta_xy / delta_z.
class CpVehicleLocalPosition : public smacc2::ISmaccComponent
{
public:
  CpVehicleLocalPosition();
  virtual ~CpVehicleLocalPosition();

  void onInitialize() override;

  float getX() const;
  float getY() const;
  float getZ() const;
  float getHeading() const;
  bool isValid() const;

  // --- global reference of the local frame ---
  bool globalRefValid() const;  // xy_global and a reference has been received
  double getRefLat() const;     // degrees
  double getRefLon() const;     // degrees
  float getRefAlt() const;      // metres AMSL
  uint64_t getRefTimestamp() const;
  uint8_t getXyResetCounter() const;
  uint8_t getZResetCounter() const;

  // lat/lon (degrees) -> local NED x (north), y (east) in metres.
  // Returns false (outputs untouched) while the global reference is not valid.
  bool projectToNed(double lat, double lon, float & x, float & y) const;
  bool reprojectFromNed(float x, float y, double & lat, double & lon) const;

  smacc2::SmaccSignal<void()> onPositionReceived_;
  smacc2::SmaccSignal<void()> onLocalPositionReset_;  // xy or z reset counter changed

private:
  void onPositionMessage(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriber_;
  float x_ = 0.0f;
  float y_ = 0.0f;
  float z_ = 0.0f;
  float heading_ = 0.0f;
  bool valid_ = false;

  bool xyGlobal_ = false;
  bool zGlobal_ = false;
  double refLat_ = 0.0;
  double refLon_ = 0.0;
  float refAlt_ = 0.0f;
  uint64_t refTimestamp_ = 0;
  uint8_t xyResetCounter_ = 0;
  uint8_t zResetCounter_ = 0;
  bool firstMessage_ = true;
  MapProjection projection_;

  mutable std::mutex mutex_;
};

}  // namespace cl_px4_mr
