#include <smacc/smacc_client_behavior.h>

namespace smacc
{
void SmaccClientBehavior::onEntry()
{
  RCLCPP_DEBUG(
    getNode()->get_logger(), "[%s] Default empty SmaccClientBehavior onEntry",
    this->getName().c_str());
}

void SmaccClientBehavior::onExit()
{
  RCLCPP_DEBUG(
    getNode()->get_logger(), "[%s] Default empty SmaccClientBehavior onExit",
    this->getName().c_str());
}
}  // namespace smacc