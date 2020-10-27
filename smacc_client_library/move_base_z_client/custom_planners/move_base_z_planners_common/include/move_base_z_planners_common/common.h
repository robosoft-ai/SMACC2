#pragma once
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Twist &msg)
{
    out << "Twist [" << msg.linear.x << "m , " << msg.linear.y << "m , " << msg.angular.z << "rad ]";
    return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Pose &msg)
{
    out << "Position [" << msg.position.x << "m , " << msg.position.y << "m , " << msg.position.z << "m ]";
    out << " Orientation [" << msg.orientation.x << " , " << msg.orientation.y << " , " << msg.orientation.z << ", " << msg.orientation.w <<"]";
    return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::PoseStamped &msg)
{
    out << msg.pose;
    return out;
}