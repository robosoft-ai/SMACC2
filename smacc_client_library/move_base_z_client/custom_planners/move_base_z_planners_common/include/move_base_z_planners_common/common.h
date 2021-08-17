#pragma once
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Twist &msg);

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Pose &msg);

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::PoseStamped &msg);

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Point &msg);

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Quaternion &msg);