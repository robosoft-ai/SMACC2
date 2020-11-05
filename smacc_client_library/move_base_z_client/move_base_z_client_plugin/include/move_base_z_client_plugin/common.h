#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <builtin_interfaces/msg/time.hpp>

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Quaternion& msg);
std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Pose& msg);
std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Point& msg);
std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::PoseStamped& msg);
std::ostream& operator<<(std::ostream& out, const nav2_msgs::action::NavigateToPose::Goal& msg);
std::ostream& operator<<(std::ostream& out, const builtin_interfaces::msg::Time& msg);
 