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

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <smacc2/smacc.hpp>
#include <string>

namespace cl_nav2z
{
class CpAmcl : public smacc2::ISmaccComponent
{
public:
  CpAmcl() {}
  virtual ~CpAmcl() {}

  inline std::string getName() const override { return "AMCL"; }

  inline void onInitialize() override
  {
    initalPosePub_ = getNode()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(10));
  }

  inline void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose)
  {
    initalPosePub_->publish(initialpose);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initalPosePub_;
};

}  // namespace cl_nav2z
