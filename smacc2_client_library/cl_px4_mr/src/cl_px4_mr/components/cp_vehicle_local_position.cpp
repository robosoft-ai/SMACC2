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
  std::lock_guard<std::mutex> lock(mutex_);
  x_ = msg->x;
  y_ = msg->y;
  z_ = msg->z;
  heading_ = msg->heading;
  valid_ = msg->xy_valid && msg->z_valid;

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

}  // namespace cl_px4_mr
