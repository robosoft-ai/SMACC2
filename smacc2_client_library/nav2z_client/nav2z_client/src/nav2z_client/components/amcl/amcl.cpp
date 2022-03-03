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
#include <nav2z_client/components/amcl/amcl.hpp>

#include <string>

namespace cl_nav2z
{
Amcl::Amcl() {}

Amcl::~Amcl() {}

std::string Amcl::getName() const { return "AMCL"; }

void Amcl::onInitialize()
{
  initalPosePub_ = getNode()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(10));
}

void Amcl::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose)
{
  initalPosePub_->publish(initialpose);
}

}  // namespace cl_nav2z
