#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class TemperatureClass
{
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::init(argc,argv, "temperature_sensor_node");
  // ros::NodeHandle nh;
  // auto pub = nh.advertise<std_msgs::Float32>("/temperature",1);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("temperature_sensor_node");

  auto pub = node->create_publisher<std_msgs::msg::Float32>("/temperature", 1);
  int i = 0;

  rclcpp::Rate r(30);

  while (rclcpp::ok())
  {
    std_msgs::msg::Float32 msg;
    // temperatures varies sinoidally from -20 to 20 degrees, but sporadically it has 32 deg
    if (i % 100 == 0)
      msg.data = 32;
    else
      msg.data = 20 * sin(0.1 * node->now().nanoseconds() / 1e-9);

    pub->publish(msg);

    r.sleep();
    // rclcpp::spinOnce();
    rclcpp::spin_some(node);
    i++;
  }
}