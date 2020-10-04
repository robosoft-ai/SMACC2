#include <smacc/smacc_updatable.h>

namespace smacc
{

ISmaccUpdatable::ISmaccUpdatable(rclcpp::Node::SharedPtr& nh)
    : nh_(nh), 
      lastUpdate_(0)
{
}

ISmaccUpdatable::ISmaccUpdatable(rclcpp::Node::SharedPtr& nh, rclcpp::Duration duration)
    : nh_(nh),
      lastUpdate_(0),
      periodDuration_(duration)
{
}

void ISmaccUpdatable::setUpdatePeriod(rclcpp::Duration duration)
{
    periodDuration_ = duration;
}

void ISmaccUpdatable::executeUpdate()
{
    bool update = true;
    if (periodDuration_)
    {
        auto now = this->nh_->get_clock()->now();
        auto ellapsed = now - this->lastUpdate_;
        update = ellapsed > *periodDuration_;
        if(update)
        {
            this->lastUpdate_ = now;
        }
    }

    if (update)
    {
        this->update();
    }
}
}