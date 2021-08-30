// Copyright 2021 RobosoftAI Inc.
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
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once
#include <chrono>
#include <optional>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

namespace smacc2
{
class ISmaccUpdatable
{
public:
  ISmaccUpdatable();
  explicit ISmaccUpdatable(rclcpp::Duration duration);

  void executeUpdate(rclcpp::Node::SharedPtr node);
  void setUpdatePeriod(rclcpp::Duration duration);

protected:
  virtual void update() = 0;

private:
  std::optional<rclcpp::Duration> periodDuration_;
  rclcpp::Time lastUpdate_;
};
}  // namespace smacc2
