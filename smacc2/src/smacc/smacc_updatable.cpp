#include <smacc/smacc_updatable.h>

namespace smacc
{
ISmaccUpdatable::ISmaccUpdatable() : lastUpdate_(0) {}

ISmaccUpdatable::ISmaccUpdatable(rclcpp::Duration duration)
: periodDuration_(duration), lastUpdate_(0)

{
}

void ISmaccUpdatable::setUpdatePeriod(rclcpp::Duration duration) { periodDuration_ = duration; }

void ISmaccUpdatable::executeUpdate(rclcpp::Node::SharedPtr node)
{
  bool update = true;
  if (periodDuration_)
  {
    auto now = node->now();
    auto ellapsed = now - this->lastUpdate_;
    update = ellapsed > *periodDuration_;
    if (update)
    {
      this->lastUpdate_ = now;
    }
  }

  if (update)
  {
    this->update();
  }
}
}  // namespace smacc