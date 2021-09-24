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

#include <move_base_z_client_plugin/common.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg)
{
  out << " Orientation [" << msg.x << " , " << msg.y << " , " << msg.z << ", " << msg.w
      << "] , yaw: " << tf2::getYaw(msg);
  return out;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Point & msg)
{
  out << "[" << msg.x << " , " << msg.y << " , " << msg.z << "]";
  return out;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg)
{
  out << " p " << msg.position;
  out << " q [" << msg.orientation.x << " , " << msg.orientation.y << " , " << msg.orientation.z
      << ", " << msg.orientation.w << "]";
  return out;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg)
{
  out << msg.pose;
  return out;
}

std::ostream & operator<<(std::ostream & out, const nav2_msgs::action::NavigateToPose::Goal & msg)
{
  out << msg.pose;
  return out;
}

std::ostream & operator<<(std::ostream & out, const builtin_interfaces::msg::Time & msg)
{
  out << "seconds: " << rclcpp::Time(msg).seconds();
  return out;
}
