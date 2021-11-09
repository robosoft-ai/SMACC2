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
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <tf2/utils.h>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg)
{
  out << " Orientation [" << msg.x << " , " << msg.y << " , " << msg.z << ", " << msg.w
      << "] , yaw: " << tf2::getYaw(msg);
  return out;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg)
{
  return out << "[serialization geometry_msgs::msg::PoseStamped]";

}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Transform & msg)
{
  return out << "[serialization geometry_msgs::msg::Transform]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg)
{
  return out << "[serialization geometry_msgs::msg::PoseStamped]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg)
{
  return out << "[serialization geometry_msgs::msg::Pose]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Vector3 & msg)
{
  return out << "[serialization geometry_msgs::msg::Vector3]";
}

std::ostream & operator<<(std::ostream & out, const moveit_msgs::srv::GetPositionIK::Request & msg)
{
  return out << "[moveit_msgs::srv::GetPositionIK::Request]";
}

std::ostream & operator<<(std::ostream & out, const sensor_msgs::msg::JointState & msg)
{
  return out << "[moveit_msgs::srv::GetPositionIK::Request]";
}
