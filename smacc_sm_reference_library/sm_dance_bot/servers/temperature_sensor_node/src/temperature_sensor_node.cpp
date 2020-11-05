#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto nh = rclcpp::Node::make_shared("temperature_sensor_node");
    auto pub = nh->create_publisher<sensor_msgs::msg::Temperature>("/temperature",1);

    rclcpp::Rate r(10);

    while(rclcpp::ok())
    {
        sensor_msgs::msg::Temperature msg;
        pub->publish(msg);

        r.sleep();
        rclcpp::spin_some(nh);
    }
}