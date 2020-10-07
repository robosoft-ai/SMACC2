#include <ros_timer_client/cl_ros_timer.h>

namespace cl_ros_timer
{

ClRosTimer::ClRosTimer(std::chrono::seconds duration, bool oneshot)
    : duration_(duration),
    oneshot_ (oneshot)
{
}

ClRosTimer::~ClRosTimer()
{
    timer_->cancel();
}

void ClRosTimer::initialize()
{
    auto clock = this->getNode()->get_clock();
    
    timer_ = rclcpp::create_timer(this->getNode(), clock, duration_, std::bind(&ClRosTimer::timerCallback, this));
}

void ClRosTimer::timerCallback()
{
    if (!onTimerTick_.empty())
    {
        this->onTimerTick_();
    }
    postTimerEvent_();

    if(oneshot_)
    {
        this->timer_->cancel();
    }
}

} // namespace cl_ros_timer
