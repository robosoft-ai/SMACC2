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

#include <mutex>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

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

  smacc2::SmaccSignal<void()> onPositionReceived_;

private:
  void onPositionMessage(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriber_;
  float x_ = 0.0f;
  float y_ = 0.0f;
  float z_ = 0.0f;
  float heading_ = 0.0f;
  bool valid_ = false;
  mutable std::mutex mutex_;
};

}  // namespace cl_px4_mr
