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

#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <move_group_interface_client/common.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg)
{
  return out << " Quaternion[" << msg.x << " , " << msg.y << " , " << msg.z << ", w:" << msg.w;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Transform & msg)
{
  return out << "Translation[" << msg.translation << "], Rotation[" << msg.rotation << "]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg)
{
  return out << "Position[" << msg.position << "], Orientation[" << msg.orientation << "]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg)
{
  return out << "[serialization geometry_msgs::msg::PoseStamped] frame_id: " << msg.header.frame_id
             << ", pose: " << msg.pose;
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Vector3 & msg)
{
  return out << "[ " << msg.x << " " << msg.y << " " << msg.z << "]";
}

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Point & msg)
{
  return out << "[ " << msg.x << " " << msg.y << " " << msg.z << "]";
}

std::ostream & operator<<(std::ostream & out, const moveit_msgs::srv::GetPositionIK::Request & msg)
{
  return out << "[moveit_msgs::srv::GetPositionIK::Request] position["
             << msg.ik_request.pose_stamped << "]";
}

std::ostream & operator<<(std::ostream & out, const sensor_msgs::msg::JointState & /*msg*/)
{
  return out << "[sensor_msgs::msg::JointState]";
}
