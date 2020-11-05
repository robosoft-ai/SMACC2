#include <move_base_z_client_plugin/common.h>
#include <rclcpp/rclcpp.hpp>

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Quaternion &msg)
{
    out << " Orientation [" << msg.x << " , " << msg.y << " , " << msg.z << ", " << msg.w <<"]";
    return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Point &msg)
{
    out << "[" << msg.x << " , " << msg.y << " , " << msg.z << "]";
    return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::Pose &msg)
{
    out << " p " << msg.position;
    out << " q [" << msg.orientation.x << " , " << msg.orientation.y << " , " << msg.orientation.z << ", " << msg.orientation.w <<"]";
    return out;
}

std::ostream& operator<< (std::ostream &out, const geometry_msgs::msg::PoseStamped &msg)
{
    out << msg.pose;
    return out;
}


std::ostream& operator << (std::ostream& out, const nav2_msgs::action::NavigateToPose::Goal& msg)
{
    out << msg.pose;
    return out;
}

std::ostream& operator<<(std::ostream& out, const builtin_interfaces::msg::Time& msg)
{
    out <<"seconds: " << rclcpp::Time(msg).seconds();
    return out;
}

