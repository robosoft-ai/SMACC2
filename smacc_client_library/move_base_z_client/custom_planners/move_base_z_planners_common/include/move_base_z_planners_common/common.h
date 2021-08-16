#pragma once
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Twist & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Pose & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::PoseStamped & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Point & msg);

std::ostream & operator<<(std::ostream & out, const geometry_msgs::msg::Quaternion & msg);
