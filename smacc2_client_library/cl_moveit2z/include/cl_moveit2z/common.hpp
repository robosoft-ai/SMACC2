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

#include <tf2/utils.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Transform & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Point & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Vector3 & msg);

std::ostream & operator<<(std::ostream & out, const moveit_msgs::srv::GetPositionIK::Request & msg);

std::ostream & operator<<(std::ostream & out, const sensor_msgs::msg::JointState & msg);
