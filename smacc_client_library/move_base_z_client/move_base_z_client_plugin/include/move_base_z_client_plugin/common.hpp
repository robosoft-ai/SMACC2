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

#pragma once

#include <iostream>

#include <tf2/transform_datatypes.h>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg);
std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg);
std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Point & msg);
std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg);
std::ostream & operator<<(std::ostream & out, const nav2_msgs::action::NavigateToPose::Goal & msg);
std::ostream & operator<<(std::ostream & out, const builtin_interfaces::msg::Time & msg);
